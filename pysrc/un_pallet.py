#!/usr/bin/env python
import rospy
from utils import navigation
import numpy as np
from math import sqrt
from time import time



class Estado:
    def __init__(self,matrix,angle_robot,orientation_pallet,nodo_padre,instrucciones,cargado,pallet_final=None,coste_accion=0,pose_pallet=None):
        self.matrix=matrix
        self.instrucciones=instrucciones
        self.coste=coste_accion
        self.padre(nodo_padre)
        self.pose_robot(angle_robot,cargado)
        self.pose_pallet(pose_pallet,orientation_pallet)
        self.padre(nodo_padre)
        self.pose_pallet_objetivo(pallet_final)
        self.estrella(coste_accion)
     
    def pose_robot(self,angle_robot,cargado):
        self.robotc=int(np.where(self.matrix==1)[1])  #calculamos la posicion del robot
        self.robotf=int(np.where(self.matrix==1)[0])
        self.robota=int(angle_robot)
        self.cargado=cargado
        """     0
                |
        270 ____|____  90 Referencia de hacia donde es la direccion de avance del robot
                |  
                |
               180   
        """
    
    def pose_pallet(self,pose,angle):
        """ | -> angle=V
            -- -> angle=H
        """
        
        if self.cargado:
            self.palletc=int(np.where(self.matrix==1)[1]) #calculamos la posicion del pallet
            self.palletf=int(np.where(self.matrix==1)[0]) 
        elif self.padrenodo!=None and not self.cargado:
            self.palletc=(self.padrenodo).palletc #calculamos la posicion del pallet
            self.palletf=(self.padrenodo).palletf 
        elif pose!=None:
            self.palletc=pose[1]
            self.palletf=pose[0]
        self.palleta=angle


    def pose_pallet_objetivo(self,pallet_final):
        if pallet_final==None: #si tiene padre es la misma que pose que la del padre
            padre=self.padrenodo #fijamos pose objetivo dejar pallet
            self.pallet_objc=padre.pallet_objc
            self.pallet_objf=padre.pallet_objf
            self.pallet_obja=padre.pallet_obja
        elif pallet_final!=None: #si no tiene padre (estado inicial) metemos los introducidos como argumentos de la instancia de clase 
            self.pallet_objc=pallet_final[1]
            self.pallet_objf=pallet_final[0]
            self.pallet_obja=pallet_final[2]


    def estrella(self,coste_accion):
        self.h=self.funcion_heuristica() #funcion heuristica
        #self.h=0 #algoritmo de dijkstra
        try: 
            self.g=int(self.padrenodo.g)+coste_accion
        except AttributeError: 
            self.g=0 # si no encuentra un nodo padre es que es el primer nodo
        self.f=self.h+self.g
        
        

    """def funcion_heuristica(self):
        #funcion heuristica distancia de Hamming
       if  self.pallet_objc==self.palletc and self.pallet_objf==self.palletf and self.palleta==self.pallet_obja: return 0
       else: return 1"""
    
    

    def funcion_heuristica(self):
        """Vamos a dividir la distancia a calcular en diferentes etapas. Es decir vamos a situar diferentes puntos en el espacio y calcularemos la suma de distancia entre ellos:
            pose_actual_robot->pose_pallet(antes de cargarlo)->pose_objetivo_pallet->pose_final_robot
        """
        estimacion=0
        
        pose_robot=(self.robotc,self.robotf) #posicion del robot
        pose_pallet=(self.palletf,self.palletc) #posicion pallet
        pose_objetivo_pallet=(self.pallet_objf,self.pallet_objc)

        estimacion=self.calculo_distancia_puntos(pose_robot,pose_pallet)
        
        estimacion=estimacion+int(self.calculo_distancia_puntos(pose_pallet,pose_objetivo_pallet))*2+int(self.calculo_distancia_puntos(pose_inicio,pose_robot))
        if self.pallet_objc==self.palletc and self.pallet_objf==self.pallet_objf:
            estimacion=0
            estimacion=int(self.calculo_distancia_puntos(pose_inicio,pose_robot))
        return estimacion
        

    def calculo_distancia_puntos(self,punto1,punto2): 
        #funcion que calcula la distancia euclidiana entre dos puntos
        #resta=(punto1[0]-punto2[0],punto1[1]-punto2[1])
        #return sqrt(int(resta[0])**2+int(resta[1])**2)

        #funcion que calcula la distancia de manhattan entre dos puntos
        return abs(punto1[0]-punto2[0])+abs(punto1[1]-punto2[1])


    def padre(self,padre):
        #definimos el nodo padre
        self.padrenodo=padre
 
    
    def __eq__(self,otro_estado):  #funcion para comparar estados
        return (np.array_equal(self.matrix,otro_estado.matrix) and self.robotc==otro_estado.robotc and self.robotf==otro_estado.robotf and self.robota==otro_estado.robota
            and self.palletc==otro_estado.palletc and self.palletf==otro_estado.palletf and self.palleta==otro_estado.palleta and self.cargado==otro_estado.cargado)



class Practica1:

    def __init__(self):
        self.nav = navigation.Navigation()
        #abrimos un archico txt con las estadisticas
        estadistica_txt=open("statistics.txt","w")
        #estadistica_txt.truncate(0); estadistica_txt.seek(0)
        t0=time() #tiempo inicio
        self.execAstar()
        t1=time()
        estadistica_txt.write("Tiempo ejecucion A*: "+str(t1-t0)+"segundos\n")
        estadistica_txt.write("Nodos expandidos: "+str(self.nodos_expandidos)+"\n")
        estadistica_txt.write("Longitud del plan:"+str(self.tamanhoplan)+"\n")
        estadistica_txt.write("Coste del plan:"+str(self.coste_total)+"\n")
        #raw_input("Pulsa tecla enter para empezar a mover")
        #self.execTXT()
        t2=time()
        estadistica_txt.write("Tiempo ejecucion instruccion en gazebo:"+str(t2-t1)+"segundos\n")
        estadistica_txt.write("Tiempo total consumido:"+str(t2-t0)+"segundos\n")

    

    def execAstar(self):
        #contador de nodos expandidos en total
        self.nodos_expandidos=0
        #abrimos archivo txt para escribir la solucion
        solucion_txt=open("plan.txt","w")
        #solucion_txt.truncate(0); solucion_txt.seek(0)
        matriz_txt=open("matriz.txt","w")
        #matriz_txt.truncate(0); matriz_txt.seek(0)
    
        #Algoritmo de busqueda en estrella
        estado_inicio,estado_final=self.estado_inicial_final() #condiciones de nodo inicio y meta
        self.abierta=[]
        self.abierta.append(estado_inicio) #anadimos el nodo inicio a abierta
        self.cerrada=[]
        exito=False
        print("Calculando algoritmo de busqueda")
        while self.abierta!=[]: 
            primer_nodo_abierta=self.abierta.pop(0)
            if primer_nodo_abierta==estado_final: exito=True; break
            
            if not (primer_nodo_abierta in self.cerrada):
                self.expandir_nodo(primer_nodo_abierta)
                self.cerrada.append(primer_nodo_abierta)
                self.nodos_expandidos+=1
               
        if exito:
            instrucciones=[]
            matriz=[]
            self.coste_total=0
            #seguimos los punteros de los diferentes estados hasta el estado inicial y vamos anhadiendo las instrucciones a la lista
            print("Exito en la busqueda de la solucion")
            while primer_nodo_abierta.instrucciones:
                instrucciones.append(primer_nodo_abierta.instrucciones) #anhadimos las instrucciones siempre al final
                matriz.append(primer_nodo_abierta.matrix) 
                self.coste_total+=primer_nodo_abierta.coste
                primer_nodo_abierta=primer_nodo_abierta.padrenodo
                
            for j in range(len(instrucciones)-1,-1,-1): #ahora le damos la vuelta a la lista y escribimos en el txt
                for i in instrucciones[j]:
                    solucion_txt.write(i+"\n")
                
            print("coste total",self.coste_total)
            print("nodos expandidos:",self.nodos_expandidos)
            for j in range(len(instrucciones)-1,-1,-1): #ahora le damos la vuelta a la lista y escribimos en el txt
                for i in instrucciones[j]:
                    matriz_txt.write(i+"\n")
                np.savetxt(matriz_txt,matriz[j],fmt="%.0f")
                matriz_txt.write("ANGLE: "+str(primer_nodo_abierta.robota)+"\n")
                matriz_txt.write("_______________________________________________\n")

            self.tamanhoplan=len(instrucciones)

        if not exito:
            print("Fracaso en la busqueda de solucion")

    
    def palletocupa3(self,estado):
        #Funcion que comprueba que el robot puede avanzar con el pallet cargado hacia delante
        if estado.robota==0 and estado.cargado:
            if estado.palleta=="H":
                if (estado.matrix[estado.robotf-1,estado.robotc-1]==3 or estado.matrix[estado.robotf-1,estado.robotc+1]==3): return False
                else: return True
            elif estado.palleta=="V":
                if estado.matrix[estado.robotf-2,estado.robotc]==3: return False
                else: return True

        elif estado.robota==90 and estado.cargado:
            if estado.palleta=="H":
                if (estado.matrix[estado.robotf,estado.robotc+2]==3): return False
                else: return True 
            if estado.palleta=="V":
                if (estado.matrix[estado.robotf+1,estado.robotc+1]==3 or estado.matrix[estado.robotf-1,estado.robotc+1]==3): return False
                else: return True
            
        elif estado.robota==180 and estado.cargado:
            if estado.palleta=="H":
                if (estado.matrix[estado.robotf+1,estado.robotc+1]==3 or estado.matrix[estado.robotf+1,estado.robotc-1]==3): return False
                else: return True
            elif estado.palleta=="V":
                if (estado.matrix[estado.robotf+2,estado.robotc]==3): return False
                else: return True
            
        elif estado.robota==270 and estado.cargado:
            if estado.palleta=="H":
                if (estado.matrix[estado.robotf,estado.robotc-2]==3): return False
                else: return True
            if estado.palleta=="V":
                if (estado.matrix[estado.robotf-1,estado.robotc-1]==3 or estado.matrix[estado.robotf+1,estado.robotc-1]==3): return False
                else: return True
        

    def palletocupa3giro(self,estado,giro):
        #Funcion que comprueba si el robot puede girar cuando el pallet esta cargado
        if (estado.palleta=="H") and estado.cargado:
            if (estado.matrix[estado.robotf-1,estado.robotc]==3 or estado.matrix[estado.robotf+1,estado.robotc]==3): return False
            else: return True

        if (estado.palleta=='V') and estado.cargado:
            if (estado.matrix[estado.robotf,estado.robotc-1]==3 or estado.matrix[estado.robotf,estado.robotc+1]==3): return False
            else: return True

        
    def expandir_nodo(self,estado):
        matrix_new=estado.matrix

        ins0=["nav.move(1)"]
        ins1=["nav.rotateRight()"]
        ins2=["nav.rotateLeft()"]
        ins3=["nav.downLift()"]
        ins4=["nav.upLift()"]
        
        if estado.robota==0 and (matrix_new[estado.robotf-1,estado.robotc]==0): #avance en 0 grados
            matrix_new0=matrix_new.copy()
            matrix_new0[estado.robotf,estado.robotc]=0
            matrix_new0[estado.robotf-1,estado.robotc]=1
            if estado.cargado: coste=2
            else: coste=1
            if estado.cargado and self.palletocupa3(estado): nuevo_estado=Estado(matrix_new0,0,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new0,0,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            
            
        if estado.robota==90 and (matrix_new[estado.robotf,estado.robotc+1]==0): #avance en 90 grados
            matrix_new1=matrix_new.copy() 
            matrix_new1[estado.robotf,estado.robotc]=0
            matrix_new1[estado.robotf,estado.robotc+1]=1
            if estado.cargado: coste=2
            else: coste=1
            if estado.cargado and self.palletocupa3(estado): nuevo_estado=Estado(matrix_new1,90,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new1,90,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            

        if estado.robota==180 and (matrix_new[estado.robotf+1,estado.robotc]==0):  #avance en 180 gradps
            matrix_new2=matrix_new.copy()
            matrix_new2[estado.robotf,estado.robotc]=0
            matrix_new2[estado.robotf+1,estado.robotc]=1
            if estado.cargado: coste=2
            else: coste=1
            if estado.cargado and self.palletocupa3(estado): nuevo_estado=Estado(matrix_new2,180,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new2,180,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
        

        if estado.robota==270 and (matrix_new[estado.robotf,estado.robotc-1]==0): #avance en 270 grados
            matrix_new3=matrix_new.copy()
            matrix_new3[estado.robotf,estado.robotc]=0
            matrix_new3[estado.robotf,estado.robotc-1]=1
            if estado.cargado: coste=2
            else: coste=1
            if estado.cargado and self.palletocupa3(estado): nuevo_estado=Estado(matrix_new3,270,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new3,270,estado.palleta,estado,ins0,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)

        if True: #giro derecha
            if estado.cargado: coste=3
            else: coste=2
            angle=estado.robota
            
            if estado.palleta=="V" and estado.cargado: nueva_orientacion="H"
            elif estado.palleta=="H" and estado.cargado: nueva_orientacion="V"
            else: nueva_orientacion=estado.palleta
            
            if estado.cargado and self.palletocupa3giro(estado,"derecha"): nuevo_estado=Estado(matrix_new,(angle+90)%360,nueva_orientacion,estado,ins1,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new,(angle+90)%360,nueva_orientacion,estado,ins1,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)

        if True: #giro izquierda
            if estado.cargado: coste=3
            else: coste=2
            angle=estado.robota
    
            if estado.palleta=="V" and estado.cargado: nueva_orientacion="H"
            elif estado.palleta=="H" and estado.cargado: nueva_orientacion="V"
            else: nueva_orientacion=estado.palleta
            
            if estado.cargado and self.palletocupa3giro(estado,"izquierda"): nuevo_estado=Estado(matrix_new,abs((angle-90)%360),nueva_orientacion,estado,ins2,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if not estado.cargado: nuevo_estado=Estado(matrix_new,abs((angle-90)%360),nueva_orientacion,estado,ins2,estado.cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)


            
        #vamos a generar estados de carga, para esto el pallet no puede estar en la pose final
        if  not estado.cargado and (estado.pallet_objc!=estado.palletc and estado.pallet_objf!=estado.palletf) and (estado.robotf,estado.robotc)==(estado.palletf,estado.palletc):
            nuevo_estado=Estado(matrix_new,estado.robota,estado.palleta,estado,ins4,True,coste_accion=3)
            self.anhadir_estado(nuevo_estado)

        #generamos instrucciones de descarga
        if estado.cargado:
            nuevo_estado=Estado(matrix_new,estado.robota,estado.palleta,estado,ins3,False,coste_accion=3)
            self.anhadir_estado(nuevo_estado)

                
        

    def anhadir_estado(self,estado_nuevo):
        """Funcion para anadir nuevos estados ordenados en abierta.
            INPUT:  nuevo estado a anadir
            Devuelve la lista con el nuevo estado en la posicion correcta
            #algoritmo de estrella tipo 2
        """
        anhadir=True
        funcion_evaluacion=estado_nuevo.f
        #comprobar que no este ni en abierta ni en cerrada
        for i in range(len(self.cerrada)): #comprobamos que no este en cerrada
            if estado_nuevo==self.cerrada[i]: anhadir=False; break
        if anhadir:   
            for i in range(len(self.abierta)): #comprobamos que no este en abierta
                if estado_nuevo==self.abierta[i]: 
                    if (self.abierta[i]).f>funcion_evaluacion: #si esta en abierta miramos si tiene mejor valor de la funcion de evaluacion
                        self.abierta.remove(self.abierta[i]) #eliminamos el antiguo de abierta
                        break
                    else:
                        anhadir=False; break

        if anhadir:       
            exito=False #si ya anadimos no hace falta anadir al final de la lista
            for i in range (len(self.abierta)):
                if funcion_evaluacion<=(self.abierta[i].f):
                    self.abierta.insert(i,estado_nuevo)
                    exito=True
                    break #anadimos y salimos del bucle
            if not exito: self.abierta.append(estado_nuevo) #si no tuvimos exito anadimos al final

    """def anhadir_estado(self,estado_nuevo):
        #algoritmo de estrella tipo 1
        funcion_evaluacion=estado_nuevo.f
        for i in range (len(self.abierta)):
            if funcion_evaluacion<=(self.abierta[i].f):
                self.abierta.insert(i,estado_nuevo)
                break #anadimos y salimos del bucle
        self.abierta.append(estado_nuevo)"""


    def estado_inicial_final(self):
        import sys
        """Funcion para localizar objetos en una matriz representativa de un mapa.
        -Localizamos el robot como un 1
        -Localizamos obtaculos/paredes/fuera de mapa como 9
        
        EJEMPLO DE MANERA DE LANZAR EL SCRIPT:
        >> python script.py 5,2 V 1,5 H"""
        global pose_inicio,pallet_inicio

        try:
            argumentos=sys.argv
            pallet_inicio=(int(argumentos[1][0]),int(argumentos[1][2]))
            pallet_inicio_orientacion=argumentos[2]
            pallet_final=(int(argumentos[3][0]),int(argumentos[3][2]))
            pallet_final_orientacion=argumentos[4]
        except:
            print ("Se han introducido mal los argumentos, para que sean correctos tienen que tener la siguiente forma")
            print(">> python un_pallet.py 5,2 V 1,5 H")   
            print(">> python script.py pose_inicial orien_inicial pose_final orien_final")  
            sys.exit()   
        
        #matriz estado inicial
        matrix=np.array(((9,9,9,9,9,9,9,9),
                        (9,9,0,0,0,0,9,9),
                        (9,9,0,0,0,0,9,9),
                        (9,9,3,3,0,0,9,9),
                        (9,0,0,0,0,0,0,9),
                        (9,0,0,0,0,0,0,9),
                        (9,0,0,0,3,3,3,9),
                        (9,9,9,0,0,0,9,9),
                        (9,9,9,0,1,0,9,9),
                        (9,9,9,9,9,9,9,9)))

        #calculamos donde esta el robot en el inicio
        pose_inicio=np.where(matrix==1)

        #definimos caracteristicas para el objetivo del pallet
        pallet_final=(int(argumentos[3][0]),int(argumentos[3][2]),pallet_final_orientacion)

        #creamos estado inicial
        A=Estado(matrix,0,pallet_inicio_orientacion,None,instrucciones=None,cargado=False,pallet_final=pallet_final,pose_pallet=pallet_inicio)
        
    
        #creamos estado final
        B=Estado(matrix,0,pallet_final_orientacion,None,instrucciones=None,cargado=False,pallet_final=pallet_final,pose_pallet=pallet_final)

        return A,B
        
 
    def execTXT(self):
        print("Ejecutamos las ordenes del txt")
        with open("plan.txt","r") as file:
            lines=file.readlines()
        for ins in lines:
            if ins=="nav.move(1)\n": self.nav.move(1)
            elif ins=="nav.rotateLeft()\n": self.nav.rotateLeft()
            elif ins=="nav.rotateRight()\n": self.nav.rotateRight()
            elif ins=="nav.downLift()\n": self.nav.downLift()
            elif ins=="nav.upLift()\n": self.nav.upLift()



if __name__ == '__main__':
    try:
        p = Practica1()
    except (RuntimeError, TypeError, NameError) as err:
        rospy.loginfo("Practica2 terminated: ", err)