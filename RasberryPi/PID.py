import numpy as np
import matplotlib.pyplot as plt
from IPython.display import display, clear_output

def leadLag(MV, Kp, Tlead, Tlag, Ts, PV, PVinit=0,method="EBD"):
    '''
    :MV: input vector
    :Kp: process gain
    :PV: experimental output vector obtained in response to the input vector MV
    :Ts: sampling period [s]   
    :PVInit: (optional: default value is 0)
    :method: discretisation method (optional: default value is 'EBD')
        EBD: Euler Backward difference
        EFD: Euler Forward difference
        TRAP: Trapezoïdal method
    '''
    if (Tlag != 0):
        K = Ts/Tlag
        if len(PV) == 0:
            PV.append(PVinit)
        else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
            if method == 'EBD':
                PV.append(((1/(1+K))*PV[-1] + ((Kp*K)/(1+K))*((1+Tlead/Ts)*MV[-1]-(Tlead/Ts)*MV[-2]))) 
            elif method == 'EFD':
                PV.append((1-K)*PV[-1] + Kp*K*((Tlead/Ts)*MV[-1] + (1-(Tlead/Ts))*MV[-2]))
            elif method == 'TRAP':
                PV.append(((2-K)/(2+K))*PV[-1] + ((Kp*K)/(2+K))*(2*((Tlead/Ts) + 1)*MV[-1] + (1 - (2*Tlead/Ts))*MV[-2]))         
            else:
                PV.append((1/(1+K)*PV[-1] + ((Kp*K)/(1+K))*(1+Tlead/Ts)*MV[-1]-(Tlead/Ts)*MV[-2])) 
    else:
        PV.append(Kp*MV[-1])
#-----------------------------------

def actionP(MVP, Kc, E, method="EBD-EBD"):
    """
    Action proportionnel du PID
    """
    if len(MVP)==0:
        MVP.append((Kc*E[-1]))
    else:
        if method == 'EBD-EBD':
            MVP.append(Kc*E[-1])
        elif method == 'TRAP-TRAP':
            MVP.append(Kc*E[-1])        
        else:
            MVP.append(Kc*E[-1])
    return MVP

def actionI(MVI, Kc, Ts, Ti, E, method="EBD-EBD"):
    """
    Action intégrale du PID
    """
    if len(MVI)==0:
        MVI.append(((Kc*Ts)/Ti)*(E[-1]))
    else:
        if method == 'EBD-EBD':
            MVI.append(MVI[-1] + ((Kc*Ts)/Ti)*(E[-1]))
        elif method == 'TRAP-TRAP':
            MVI.append(MVI[-1] + ((Kc*Ts)/(2*Ti))*(E[-1] + E[-2]))        
        else:
            MVI.append(MVI[-1] + ((Kc*Ts)/Ti)*(E[-1]))
    return MVI

def actionD(MVD,Tfd, Ts, Kc, Td, E, method="EBD-EBD"):
    """
    Action derivative du PID
    """
    if len(MVD)==0:
        MVD.append(((Kc*Td)/(Tfd+Ts))*(E[-1]))
    else:
        if method == 'EBD-EBD':
            MVD.append((Tfd/(Tfd+Ts))*MVD[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1] - E[-2])) 
        elif method =='TRAP-TRAP':
            MVD.append(((Tfd-(Ts/2))/(Tfd+(Ts/2)))*MVD[-1] + ((Kc*Td)/(Tfd+(Ts/2)))*(E[-1] - E[-2]))     
        else:
            MVD.append((Tfd/(Tfd+Ts))*MVD[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1] - E[-2]))
    return MVD

def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVmin, MVmax, MV, MVP, MVI, MVD, E, ManFF=False, PVinit=0, method ="TRAP-TRAP"):
    """
    SP = setpoint vector
    PV = process value vector
    Man = manual controller mode vector : bool
    MVMan = Manual value for MV vector
    MVFF = feedforward vector
    Kc = controller gain
    Ti= integral time constant
    Td = derivative time constant
    alpha = Tfd = alpha*Td = where Tfd is derivative filter time constant
    MVMin = min value for MV
    MVMax = max value for MV
    MV = Maniplated value vector
    MVP = proportional part of MV vector
    MVI =  integral part of MV vector
    MVD = derivative part of MV vector
    E = control error vector
    ManFF = activated FF in manuel mode
    PVInit = initial value of PV
    Method : discreditisation value for PV
        EBD-EBD: Euler Backward difference
        TRAP-TRAP: Trapezoïdal method
    The function PID_RT appends new values to the vector MV, MVP, MVI, MVD based on PID algorithm, controller mode and FF
    """

    #initialisation 
    Tfd = alpha*Td
    if len(PV) == 0:
        E.append(SP[-1] - PVinit)
    else: 
        E.append(SP[-1] - PV[-1])
    
    #calcul parts P,I,D
    # MV[k] is MV[-1] and MV[k-1] is MV[-2]
    if method == 'EBD-EBD':
        MVP = actionP(MVP, Kc, E, "EBD-EBD")
        MVI = actionI(MVI, Kc, Ts, Ti, E, method="EBD-EBD")
        MVD = actionD(MVD,Tfd, Ts, Kc, Td, E, method="EBD-EBD")
    elif method == 'TRAP-TRAP':
        MVP = actionP(MVP, Kc, E, "TRAP-TRAP")
        MVI = actionI(MVI, Kc, Ts, Ti, E, method="TRAP-TRAP")
        MVD = actionD(MVD,Tfd, Ts, Kc, Td, E, method="TRAP-TRAP")

    #Manual mode + anti-wind up (integrator help)
    if Man[-1] == True:
        if ManFF:
            MVI[-1] = MVMan[-1] - MVP[-1] - MVD[-1] 
        else:
            MVI[-1] = MVMan[-1] - MVP[-1] - MVD[-1] - MVFF[-1]

    #limitation 
    if (MVP[-1] + MVI[-1] + MVD[-1] + MVFF[-1]) > MVmax:
        MVI[-1] = MVmax - MVP[-1] - MVD[-1] - MVFF[-1]
    elif (MVP[-1] + MVI[-1] + MVD[-1] + MVFF[-1]) < MVmin:
        MVI[-1] = MVmin - MVP[-1] - MVD[-1] - MVFF[-1]

    #rajout
    MV.append(MVP[-1] + MVI[-1] + MVD[-1] + MVFF[-1])
#-----------------------------------

def IMCTuning(K, Tlag1, Tlag2=0, theta=0, gamma = 0.5, process="FOPDT-PI"):
    """
    IMCTuning computes the IMC PID tuning parameters for FOPDT and SOPDT processes.
    K: process gain (Kp)
    Tlag1: first (main) lag time constant [s]
    Tlag2: second lag time constant [s]
    theta: delay [s]
    gamma: used to computed the desired closed loop time constant Tclp [s] (range [0.2 -> 0.9])
    process:
        FOPDT-PI: First Order Plus Dead Time for P-I control (IMC tuning case G)
        FOPDT-PID: First Order Plus Dead Time for P-I-D control (IMC tuning case H)
        SOPDT :Second Order Plus Dead Time for P-I-D control (IMC tuning case I)
        
    return : PID controller parameters Kc, Ti and Td
    """
    Tclp = gamma*Tlag1 
    if process=="FOPDT-PI":
        Kc = (Tlag1/(Tclp+theta))/K
        Ti = Tlag1
        Td = 0
    elif process=="FOPDT-PID":
        Kc= ((Tlag1 + theta/2)/(Tclp + theta/2))/K
        Ti = Tlag1 + theta/2
        Td = (Tlag1*theta)/(2*Tlag1+theta)
    elif process=="SOPDT": 
        Kc = ((Tlag1 + Tlag2)/(Tclp + theta))/K
        Ti = (Tlag1 +Tlag2)
        Td = ((Tlag1*Tlag2))/(Tlag1+Tlag2)
    else:
        Kc = (Tlag1/(Tclp+theta))/K
        Ti = Tlag1
        Td = 0
    return (Kc, Ti, Td)


#First Order Plus Dead Time 
def Compute_Time(P,Q,omega):
    """
    Sert à faire des comparaisons entre deux process dans le domaine temporel
    :P est un premier process
    :Q est un second process
    """
    Kp,Kq = P.parameters['Kp'], Q.parameters['Kp']
    Tp,Tq = P.parameters['Tlag1'],Q.parameters['Tlag1']
    theta_p, theta_q = P.parameters['theta'],Q.parameters['theta']
    t = np.arange(0,500,0.1)
    Y1_step = ((Kp)*(1-(np.exp(-((t-theta_p)/Tp)))))
    Y2_step = ((Kq)*(1-(np.exp(-((t-theta_q)/Tq)))))
    #print(Ystep)
    fig = plt.figure()
    a1 = fig.add_axes([0,0,1,1])
    a1.plot(t,Y1_step,color="red")#Kp*(1-np.exp(-t*T))
    a1.plot(t,Y2_step,color="blue")#Kp*(1-np.exp(-t*T))
    plt.xlabel("Time [s]")
    plt.ylabel("Gain")
    a1.set_title("Time domain comparaison")

    
        
        
def Compute_All_Methods(B,G,S,omega,Strejc_order,Show=True):

    """
    Afficher les 3 modèles d'approximations dans un diagramme fréquentiel de Bode
    :B : est le modèle du premier ordre, celui de Broida 
    :G : est le modèle du second ordre, celui de Grinten 
    :S : est le modèle du n-ieme ordre, celui de Strejc 
    :omega : le vecteur de fréquence 
    :show : permet d'afficher le diagramme de Bode
    
    Ces modèles doivent être fourni comme des objets de la classe Process
    """
    s = 1j*omega
    
    Btheta ,Gtheta= np.exp(-B.parameters['theta']*s), np.exp(-G.parameters['theta']*s)
    BGain,  GGain = B.parameters['Kp']*np.ones_like(Btheta),G.parameters['Kp']*np.ones_like(Gtheta)
    BLag1,  GLag1 = 1/(B.parameters['Tlag1']*s + 1),1/(G.parameters['Tlag1']*s + 1)
    BLag2,  GLag2 = 1/(B.parameters['Tlag2']*s + 1),1/(G.parameters['Tlag2']*s + 1)
    BLead1, GLead1 = B.parameters['Tlead1']*s + 1,G.parameters['Tlead1']*s + 1
    BLead2, GLead2 = B.parameters['Tlead2']*s + 1,G.parameters['Tlead2']*s + 1

    
    Bs,Gs = np.multiply(Btheta,BGain),np.multiply(Gtheta,GGain)
    Bs,Gs = np.multiply(Bs,BLag1),np.multiply(Gs,GLag1)
    Bs,Gs = np.multiply(Bs,BLag2),np.multiply(Gs,GLag2)
    Bs,Gs = np.multiply(Bs,BLead1),np.multiply(Gs,GLead1)
    Bs,Gs = np.multiply(Bs,BLead2),np.multiply(Gs,GLead2)

    theta = S.parameters['theta']
    Kp = S.parameters['Kp']
    Tstrejc = S.parameters['Tlag1']
    Ts = (Kp*np.exp(-theta*s))/((Tstrejc*s+1)**Strejc_order)


    if Show == True:
    
        fig, (ax_gain,ax_phase) = plt.subplots(2,1)
        fig.set_figheight(12)#12
        fig.set_figwidth(22)#22

        # Graphe dans le domaine fréquentiel
        
        ax_gain.semilogx(omega,20*np.log10(np.abs(Bs)),label='Broida(s)')
        ax_gain.semilogx(omega,20*np.log10(np.abs(Gs)),label='Grinten(s)')
        ax_gain.semilogx(omega,20*np.log10(np.abs(Ts)),label='Strejc(s)')
        ax_gain.semilogx(omega,20*np.log10(np.abs(BGain)),label='Pgain')

        gain_min = np.min(20*np.log10(np.abs(Bs)/5))
        gain_max = np.max(20*np.log10(np.abs(Bs)*5))
        ax_gain.set_xlim([np.min(omega), np.max(omega)])
        ax_gain.set_ylim([gain_min, gain_max])
        ax_gain.set_ylabel('Amplitude |P| [db]')
        ax_gain.set_title("Bode diagram of 3 methods")
        ax_gain.legend(loc='best')


        # graphe de phase
        ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Bs)),label='Broida(s)')
        ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Gs)),label='Grinten(s)')
        ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Ts)),label='Strejc(s)')
        ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(BGain)),label='Pgain')   
        ax_phase.set_xlim([np.min(omega), np.max(omega)])
        ph_min = np.min((180/np.pi)*np.unwrap(np.angle(Bs))) - 10
        ph_max = np.max((180/np.pi)*np.unwrap(np.angle(Bs))) + 10
        ax_phase.set_ylim([np.max([ph_min, -200]), ph_max])
        ax_phase.set_ylabel(r'Phase $\angle P$ [°]')
        ax_phase.legend(loc='best')
    else:
        return (Bs,Gs,Ts)

def margin(Ps,C,omega,Show=True):
    """
    Elle calcule les gains de phase et de marge
    :Ps : est un process
    :C: est la fonction de transfert du Contrôleur 
    :omega : vecteur de la fréquence 
    :show : pour afficher un diagramme de Bode de la phase et du gain avec la marge représentée 
    """
    # GAIN and PHASE Margin
    s = 1j*omega
    Kc = C.parameters['Kc']
    Ti = C.parameters['Ti']
    Td = C.parameters['Td']
    Tfd = C.parameters['Tfd']
    Cs = Kc*(1 + 1/(Ti*s)+ (Td*s)/(Tfd*s +1))
    Ls =Cs*Ps


    # il faut plot Ls
    if Show ==True:
        fig, (ax_freq, ax_time) = plt.subplots(2,1)
        fig.set_figheight(12)
        fig.set_figwidth(22)

        #Amplitude
        ax_freq.semilogx(omega,20*np.log10(np.abs(Ls)),label='L(s)')
        a = 'hello'
        
        gain_min = np.min(20*np.log10(np.abs(Ls)/5))
        gain_max = np.max(20*np.log10(np.abs(Ls)*5))
        ax_freq.set_xlim([np.min(omega), np.max(omega)])
        ax_freq.set_ylim([gain_min, gain_max])
        ax_freq.set_ylabel('Amplitude |P| [db]')
        ax_freq.set_title('Bode plot of P')
        ax_freq.legend(loc='best')

            
        #Phase
        ax_time.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Ls)),label='L(s)')

        ax_time.set_xlim([np.min(omega), np.max(omega)])
        ph_min = np.min((180/np.pi)*np.unwrap(np.angle(Ps))) - 10
        ph_max = np.max((180/np.pi)*np.unwrap(np.angle(Ps))) + 10
        ax_time.set_ylim([np.max([ph_min, -200]), ph_max])
        ax_time.set_ylabel(r'Phase $\angle P$ [°]')
        ax_time.legend(loc='best')

        ax_freq.axhline(y=0,color='black')
        ax_time.axhline(y=-180,color='black')



    #FOR Crossover frequency
    i = 0
    for value in Ls:
        i+=1
        dB = 20*np.log10(np.abs(value))
        if dB < 0.05 and dB > -0.05:
            OmegaC =  omega[i-1]
            PhaseC = np.angle(value,deg=True)
            break        
    #print('Crossover frequency:',OmegaC, ', corresponding phase :',PhaseC)


    #FOR Ultimate Frequency
    n = 0
    for value in Ls:
        n+=1
        deg = np.angle(value,deg=True)
        if deg < -179.5 and deg > -180.5:
            OmegaU = omega[n-1]
            u_freq = 20*np.log10(np.abs(value))
            break
            



    print('Gain margin :',-u_freq,'dB at the ultimate frequency :',OmegaU,'rad/s')

    
    # on reprente les marges de gain et de phase
    if Show ==True:
        ax_freq.plot([OmegaU,OmegaU],[0,u_freq]) 
        ax_time.plot([OmegaC,OmegaC],[PhaseC,-180])

    print('Phase margin : ',PhaseC +180,'° at the crossover frequency :',OmegaC,'rad/s')
