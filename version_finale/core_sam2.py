from __future__ import division
import seabreeze.spectrometers as sb
import motor as m
import time 
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d



def Integration_time():
    '''This function find the integration time of an incident light beam such that the ratio signal/noise is high but without saturation'''
    pas=1 
    T=10 #in microseconds. Minimal integration time accepted by the spectro. 
    pasSat = True 
    # The principle of the research is that while the maximum of the intensity is lower than a threshold previously determined for this spectro you continue increasing T with a step of pas microseconds. Since it could be quite long every 100 steps you increase pas by ten. 
    while pasSat :
        n_iter=0
        pas*=10
        print(pas)
        while n_iter<100:
            spec = sb.Spectrometer.from_serial_number()
            spec.integration_time_micros(T)
            I_max=np.max(spec.intensities())
            spec.close()
            if  I_max > 50000:
                pasSat = False
                n_iter=10**30
            else:
                n_iter+=1
                T+=pas
                if T>10**6: #condition that allows to get out from the loop while. This is particularly useful when there is no signal because the loop would last forever in that case. 
                    raise ValueError('Could not find the optimal T ... Is there a signal in the first place ? Are the fibers truly aligned ? ')
            time.sleep(0.01) # Allows the spectro to finish its measurement and to send it to the computer. Else very weird bugs appear.
        T-=pas
    return T 

def I_mean(N,T):
    '''This function Average the intensity measured by the spectro over N measurements for each lambda. T is the integration time of the spectro. The optimal one can be determined by Integration_time()'''
    I=[0 for k in range(N)]
    spec = sb.Spectrometer.from_serial_number()
    Lambdas=spec.wavelengths()
    spec.close()
    for k in range(N):
        spec = sb.Spectrometer.from_serial_number()
        spec.integration_time_micros(T)
        I[k]=spec.intensities()
        spec.close()
        time.sleep(0.01)
    Imean=np.mean(I,axis=0)
    return Lambdas,Imean 

#def Interpolation(Lambdas,Imean,n=25):
#    return Lambdas[int(n/2):len(Lambdas)-(int(n/2))],moving_average(Imean,n)
    
def moving_average(I, n, Lambdas) :
    '''Allows to remove the high frequencies of the signl (= the noise)'''
    I_averaged = np.cumsum(I, dtype=float)#Return the cumulative sum of A 
    I_averaged[n:] = I_averaged[n:] - I_averaged[:-n]
    Lambdas_same_size = Lambdas[int(n/2):len(Lambdas)-(int(n/2))]
    I_averaged = I_averaged[n - 1:] / n
    return Lambdas_same_size,I_averaged

def I_0(LAMBDAS,T,N=100,n=25):
    '''return an array with the intensities of the light of the source between lambda_i and lambda_f every 5nm if step=5nm'''
    Lambdas_tot,Imean=I_mean(N,T)#First average I over N measurement for each lambda
    Lambdas_tot,I_averaged = moving_average(Imean, n, Lambdas_tot)#Then remove the noise by doing a moving averaging of the signal
    I_function=interp1d(Lambdas_tot,I_averaged)#Linear interpolation is the default one here
    return LAMBDAS,I_function(LAMBDAS)

def I_normalised(LAMBDAS,I0,N=100,n=25):
    '''return an array with the normalised intensities of the light exiting the Mueller Ellipsometer at each Lambdas[k] '''
    Lambdas_tot,Imean=I_mean(N,T)
    Lambdas_tot,I_averaged = moving_average(Imean, n, Lambdas_tot)
    I_function=interp1d(Lambdas_tot,I_averaged)#Linear interpolation is the default one here but something more clever could probably be done 
    return I_function(LAMBDAS)/I0

def Light_Detect2(moteur,T,LAMBDAS,alpha=10,ID=0):
    "Function that find around the initial current position the incident light beam"
    pas_alpha=int(moteur.conversion_angle_interval(alpha ,ID))
    cp=moteur.get_current_position(ID)
    gp_max=cp+pas_alpha # attention 
    Find = False 
    pas=12
    while abs(pas)>=4 and Find==False:
        if pas >0:
            test= (moteur.get_current_position(ID)-gp_max)<=0
        if pas <0:
            test= (moteur.get_current_position(ID)-cp)>=0
        while Find == False and test :
            moteur.move_motor3(ID,pas)
            spec = sb.Spectrometer.from_serial_number()
            spec.integration_time_micros(T)
            Y=spec.intensities()
            spec.close()
            if np.max(Y)>10000:
                Find=True
        if (moteur.get_current_position(ID)-gp_max)>0:
            gp_max_angle = moteur.conversion_interval_angle(gp_max ,ID)
            moteur.move_motor(ID,gp_max_angle)
            pas-=4
    if pas==0:
        raise Error('I cannot find the light beam. Check the alignement of the optical fibers. Is there a signal ? ')
    print("Done") 
    
    return Y 

def Light_Detect(moteur,T,LAMBDAS,pas=10,alpha=10,ID=0):
    "Function that finds the light spot"
    pas_alpha=int(moteur.conversion_angle_interval(alpha ,ID))
    cp=moteur.get_current_position(ID)
    gp_max=cp-pas_alpha # Maximum goal position
    Find = False 
    #the principle of the research is to continue "decreasing" the current position of the detector as long as the threshold is not reached.
    # there is another condition in case that no signal was detected between the initial angle Theta_i of the detector and Theta_i+alpha 
    while Find == False and (moteur.get_current_position(ID)-gp_max)>=0 :
        moteur.move_motor3(ID,pas)
        print(moteur.get_current_position(ID))
        spec = sb.Spectrometer.from_serial_number()
        spec.integration_time_micros(T)
        Y=spec.intensities()
        Y=list(Y)
        Y.remove(Y[3088])#at this specific lambda the spectro have a non negligible noise that can be problematic
        Y=np.array(Y)
        spec.close()
        print(np.max(Y))
        if np.max(Y)>7000:
            Find=True
    if Find == True:
        print("Done")        
        return Y 
    else:
        raise ValueError("No signal was detected ... Check the alignement of the optical fibers. Is there a signal ?")
    
def Light_Detect3(moteur,T,LAMBDAS,threshold,pas=10,alpha=10,ID=0,N=10,n=25):
    "Another version of Light Detect that takes a bit longer to run but is more suitable and robust when the intensity of the signal is low ie the ration signal/noise low."
    pas_alpha=int(moteur.conversion_angle_interval(alpha ,ID))
    cp=moteur.get_current_position(ID)
    gp_max=cp-pas_alpha # attention 
    Find = False 
    while Find == False and (moteur.get_current_position(ID)-gp_max)>=0 :
        moteur.move_motor3(ID,pas)
        print(moteur.get_current_position(ID))
        inutile,I=I_0(LAMBDAS,T,N,n)# Here we apply the threshold condition on a smoothed signal 
        if np.max(I)>threshold:
            Find=True
    if Find == True:
        print("Done")        
        return I 
    else:
        raise ValueError("No signal was detected ... Check the alignement of the optical fibers or decrease the threshold.")

def Light_Detect4(moteur,T,LAMBDAS,threshold,pas=10,alpha=10,ID=0,N=10,n=25):
    "Another version of Light Detect3 but more advanced and more time consuming."
    pas_alpha=int(moteur.conversion_angle_interval(alpha ,ID))
    cp=moteur.get_current_position(ID)
    gp_max=cp-pas_alpha  
    Find = False 
    while Find == False and (moteur.get_current_position(ID)-gp_max)>=0 :
        moteur.move_motor3(ID,pas)
        print(moteur.get_current_position(ID))
        inutile,I=I_0(LAMBDAS,T,N,n)
        # Here we apply the threshold condition on a smoothed signal 
        max_I=np.max(I)
        if max_I>threshold:
            Find=True
    
    if Find == True:
        print("in checking ...")
        check= False
        while check == False and (moteur.get_current_position(ID)-gp_max)>=0:
            moteur.move_motor3(ID,pas) 
            inutile,I_test=I_0(LAMBDAS,T,N,n)
            max_Itest=np.max(I_test)
            print(max_I,max_Itest)
            if max_I<max_Itest:
                print("the precedent position was the wrong one")
                I=I_test
                max_I=max_Itest
            else:
                check = True 
                print("the precedent position was the right one")        
        return I 
    else:
        raise ValueError("No signal was detected ... Check the alignement of the optical fibers or decrease the threshold.")
        
def Auto_transmission(moteur,T,LAMBDAS,I0,threshold=4500):
    '''Function that executes automatically all the intensity measurements required for getting the mueller matrices in transmission'''
    #Both Sam's arms are supposed to be at Theta=300 
    I_tot=[]
    angle1=210
    moteur.move_motor(1,angle1,25)#speed=25, align the arms respectively at theta1=210 degrees (emetor) ...
    moteur.move_motor(0,300,25)#And theta0=300 degrees (receptor)
    I=Light_Detect3(moteur,T,LAMBDAS,threshold,-5,90)#Move the receptor (arm 0 ) such that it finds the light spot
    pas_moteur=+17 # this value corresponds of a step of 5 degrees in angle 
    pas_angle=moteur.conversion_interval_angle(pas_moteur ,0)
    print(pas_angle)
    while angle1<=280:
        I_tot.append(I/I0)
        moteur.move_motor3(1,pas_moteur,10)#speed=10
        I=Light_Detect4(moteur,T,LAMBDAS,threshold,-5,10)
        angle1+=pas_angle
    return I_tot

def Auto_reflexion(moteur,T,LAMBDAS,I0,threshold=4500):
    '''Function that executes automatically all the intensity measurements required for getting the mueller matrices in reflexion'''
    #Both Sam's arms are supposed to be at Theta=300
    I_tot=[]
    moteur.move_motor(1,295,25)
    moteur.move_motor(0,200,25)
    I=Light_Detect3(moteur,T,LAMBDAS,threshold,5,10)
    angle1=295
    pas_moteur=-17 #corresponds to 5 degrees
    pas_angle=moteur.conversion_interval_angle( pas_moteur ,0)
    print(pas_angle)
    while angle1>=230:
        I_tot.append(I/I0)
        moteur.move_motor3(1,pas_moteur,10)
        I=Light_Detect4(moteur,T,LAMBDAS,threshold,-5,10)
        angle1+=pas_angle
    return I_tot


def calibration(moteur,LAMBDAS):
    moteur.move_motor(1,300,25)
    moteur.move_motor(0,208,25)
    input("Check the alignement of the optical fibers. Answer 1 if yes.")
    T=Integration_time()
    print(T)
    X,Y=I_0(LAMBDAS,T,10,25)
    plt.plot(X,Y)
    return T,Y


#Optionnel 
def calibration2(moteur,LAMBDAS):
    moteur.move_motor(1,210,25)
    moteur.move_motor(0,295,25)
    input("Check the alignement of the optical fibers. Answer 1 if yes.")
    T=Integration_time()
    print(T)
    X,Y=I_0(LAMBDAS,T,10,25)
    plt.plot(X,Y)
    input("Put the sample on the platform. Check its horizontality and verticality. The spot light must be inside the optical fibers both in transmission and in reflexion. Answer 1 when it is done. ")
    l1,It=I_0(LAMBDAS,T,10)
    plt.plot(l1,It)
    #calibration reflexion
    moteur.move_motor(0,165,25)
    moteur.move_motor(1,255,25)
    input("Is it aligned ? Answer 1 when it is done.")
    l2,Ir=I_0(LAMBDAS,T,10)
    plt.plot(l2,Ir)
    return 'yeah'


### Data analysis (from I to Mueller matrices). Not yet included in the code 

def Linear_Polariser(teta):
    L1=[1,np.cos(2*teta),np.sin(2*teta),0]
    L2=[np.cos(2*teta),np.cos(2*teta)**2,np.sin(2*teta)*np.cos(2*teta),0]
    L3=[np.sin(2*teta),np.sin(2*teta)*np.cos(2*teta),np.sin(2*teta)**2,0]
    L4=[0,0,0,0]
    return np.array([L1,L2,L3,L4])

def Generator():
    S_0=np.array([[1],[0],[0],[0]])
    W=np.array([np.dot(Linear_Polariser(0),S_0),np.dot(Linear_Polariser(np.pi/4),S_0),np.dot(Linear_Polariser(np.pi/2),S_0),np.dot(Linear_Polariser(3*np.pi/4),S_0)])
    return W

def Analyser():
    L1=[1,1,0,0]
    L2=[1,0,1,0]
    L3=[1,-1,0,0]
    L4=[1,0,0,1]
    return np.array([L1,L2,L3,L4])
    
    
def Transmittance_linear_polariser(LAMBDAS):
    '''return the transmittance of the linear polarizer according to lambda - source from the constructor'''
    Lambdas = np.array([399.691, 400, 401.893, 403.827, 404.901, 407.359, 409.972, 413.97, 420.434, 426.284, 433.521, 441.375, 449.845, 460.317, 471.868, 484.96, 499.591, 508.371, 526.854, 544.106, 558.739, 573.065, 589.238, 619.737, 644.382, 661.633, 677.96, 695.058, 712.617, 721.858, 730.175, 736.643, 742.033, 748.193, 750.154])
    Tr = 10**-2*np.array([16.9374, 17.6136, 20.4548, 23.1824, 25.3985, 27.9963, 30.31, 33.1922, 35.75, 37.212, 38.4307, 39.2437, 40.0973, 40.9513, 41.6837, 42.3352, 43.0275, 43.313, 43.6001, 43.7246, 43.7676, 43.8105, 43.8131, 43.9804, 44.2685, 44.596, 44.9639, 45.1696, 45.6189, 45.9451, 46.4334, 46.8403, 47.3688, 47.938, 48.091])
    z = np.polyfit(Lambdas,Tr,15)#here we choose to fit by a polynome because there was an intrinsic noise due to the numerisation of the graph taken from the constructor
    Tr_function=np.poly1d(z)
    #plt.scatter(Lambdas,Tr_function(Lambdas))
    return Tr_function(LAMBDAS)


def Mueller(I_TOT):
    F=Analyser()
    F_inv=np.linalg.inv(F)
    W=Generator()
    W_inv=np.linalg.inv(W)
    M_tot = [0 for k in range(len(I_TOT))] #[M1,M2,...,Mn] with M_i the mueller matrices at different wavelength at ONE angle. M1=[m1,m2,...,mn] with m1 muller matrice (4*4) at theta and lambda fix 
    Tr_LP=Transmittance_linear_polariser(lambda_i,lambda_f,step)
    for theta_i in range(len(I_TOT)):
        M_i=[0 for k in range(len(I_TOT)[0])]
        for lambda_i in range(len(I_TOT[0])):
            M_i[lambda_i]=1/(Tr_LP[lambda_i])**2 *np.dot(F_inv,np.dot(I_TOT[theta_i][lambda_i],W_inv))
        M_tot[theta_i]=M_i
    return M_tot

                                                         
def Get_Mueller(theta_o,lambda_o,THETAS,LAMBDAS,M_tot):
    try:
        index_theta=THETAS.index(theta_o)
    except :
        print("The theta given to the function was not in the data measure. Change the theta you want or acquire a new set of THETAS.")
    try:
       index_lambda=LAMBDAS.index(lambda_o) 
    except :
        print("The lambda given to the function was not in the data measure. Change the lambda you want or acquire a new set of THETAS.")
        
    return M_tot[index_theta][index_lambda]

