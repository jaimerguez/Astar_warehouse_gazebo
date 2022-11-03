#!/usr/bin/env python
import rospy
from utils import navigation
import numpy as np
from math import fabs, sqrt
from time import time
import copy
import sys



class Estado:
    def __init__(self,matrix,angle_robot,nodo_padre,instrucciones,pactual=None,pallet_cargado=None,pallet_final=None,coste_accion=0):
        self.matrix=matrix
        self.instrucciones=instrucciones
        self.coste=coste_accion
        self.padre(nodo_padre)
        self.pose_robot(angle_robot)
        self.pose_pallet(pactual,pallet_cargado)
        self.pose_pallet_objetivo(pallet_final)
        self.estrella(coste_accion)
        #self.heuristica()

     
    def pose_robot(self,angle_robot):
        self.robotc=int(np.where(self.matrix==1)[1])  #calculamos la posicion del robot
        self.robotf=int(np.where(self.matrix==1)[0])
        self.robota=int(angle_robot)
    
        """     0
                |
        270 ____|____  90 Referencia de hacia donde es la direccion de avance del robot
                |  
                |
               180   
        """
    
    def pose_pallet(self,pactual,pallet_cargado):
        """ | -> angle=V
            -- -> angle=H
        """
        self.pactual=pactual
        self.pallet_cargado=pallet_cargado
        if pallet_cargado==None:
            self.pallet_cargado=None
        
        else:
            self.pallet_cargado=int(pallet_cargado)
            #si esta cargado actualizamos la posicion del pallet con la del robot
            self.pactual[int(self.pallet_cargado)][1]=self.robotc #calculamos la posicion del pallet
            self.pactual[int(self.pallet_cargado)][0]=self.robotf 
         

    def pose_pallet_objetivo(self,pallet_final):
        if pallet_final==None: #si tiene padre es la misma que pose que la del padre
            padre=self.padrenodo #fijamos pose objetivo dejar pallet
            self.pallet_obj=(self.padrenodo).pallet_obj
        elif pallet_final!=None: #si no tiene padre (estado inicial) metemos los introducidos como argumentos de la instancia de clase 
            self.pallet_obj=pallet_final


    def estrella(self,coste_accion):
        self.h = self.heuristica()
        #self.h=0 #algorimto Dijkstra
        try: 
            self.g=int(self.padrenodo.g)+coste_accion
        except AttributeError: 
            self.g=0
        self.f=self.h+self.g

    def distancia_hamming(self):
        cont=0
        for i,pallet in enumerate(self.pactual):
            #comprobamos que el pallet esta encima del robot y que no esta en la posicion final donde hay que dejarlo
            if  not (pallet)==(self.pallet_obj[i]):
                #print(pose_robot,pallet[0],pallet[1])
                cont+=1
        return cont


    def heuristica(self):
        cont=0
        for i,pallet in enumerate(self.pactual):
            punto1=[pallet[0],pallet[1]]
            punto2=[self.pallet_obj[i][0],self.pallet_obj[i][1]]
            cont+=self.calculo_distancia_puntos(punto1,punto2)

        return cont
    


    def calculo_distancia_puntos(self,punto1,punto2): #funcion que calcula la distancia euclidiana entre dos puntos
        #resta=(punto1[0]-punto2[0],punto1[1]-punto2[1])
        #return sqrt(int(resta[0])**2+int(resta[1])**2)

        #funcion que calcula la distancia de manhattan entre dos puntos
        return abs(punto1[0]-punto2[0])+abs(punto1[1]-punto2[1])
   


    def padre(self,padre):
        self.padrenodo=padre
 
    
    def __eq__(self,otro_estado):  #funcion para comparar estados
        return (self.robotc==otro_estado.robotc and self.robotf==otro_estado.robotf and self.robota==otro_estado.robota 
        and self.pactual==otro_estado.pactual and self.pallet_cargado==otro_estado.pallet_cargado)



class Practica1:

    def __init__(self):
        self.nav = navigation.Navigation()
        #abrimos un archico txt con las estadisticas
        estadistica_txt=open("statistics.txt","w")
        #estadistica_txt.truncate(0); estadistica_txt.seek(0)
        t0=time() #tiempo inicio
        print("Calculando algoritmo de busqueda")
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


    def imprimir(self,estado):
        print("MATRIX")
        print(estado.matrix)
        print("Pose robot",estado.robotf,estado.robotc,estado.robota)
        print("Posicion pallet",estado.pactual,estado.pallet_cargado)
        print("Pallet    final",estado.pallet_obj)
        print("Instrucciones:",estado.instrucciones)
        print("Coste Heuristica: ",estado.h)
        print("_____________________________")
        print("")
    

    def execAstar(self):
        #abrimos archivo txt para escribir la solucion
        self.nodos_expandidos=0
        solucion_txt=open("plan.txt","w")
        solucion_txt.truncate(0); solucion_txt.seek(0)
        matriz_txt=open("matriz.txt","w")
        matriz_txt.truncate(0); matriz_txt.seek(0)
    
        #Algoritmo de busqueda en estrella
        estado_inicio,estado_final=self.estado_inicial_final() #condiciones de nodo inicio y meta
        self.abierta=[estado_inicio] #anadimos el nodo inicio a abierta
        self.cerrada=[]
        exito=False
        
       #while self.abierta!=[] or exito:
        while self.abierta!=[]:
            primer_nodo_abierta=self.abierta.pop(0)
            if primer_nodo_abierta==estado_final: exito=True;break
            
            
            if not (primer_nodo_abierta in self.cerrada):
                #self.imprimir(primer_nodo_abierta)
                self.expandir_nodo(primer_nodo_abierta)
                self.cerrada.append(primer_nodo_abierta)
                self.nodos_expandidos+=1
                

        if exito:
            instrucciones=[]
            matriz=[]
            self.coste_total=primer_nodo_abierta.g
            #seguimos los punteros de los diferentes estados hasta el estado inicial y vamos anhadiendo las instrucciones a la lista
            print("Exito en la busqueda de la solucion")
            while primer_nodo_abierta.instrucciones:
                instrucciones.append(primer_nodo_abierta.instrucciones) #anhadimos las instrucciones siempre al final
                matriz.append(primer_nodo_abierta.matrix) 
                primer_nodo_abierta=primer_nodo_abierta.padrenodo
                
            for j in range(len(instrucciones)-1,-1,-1): #ahora le damos la vuelta a la lista y escribimos en el txt
                for i in instrucciones[j]:
                    solucion_txt.write(i+"\n")
                
            print("coste total",self.coste_total)
            for j in range(len(instrucciones)-1,-1,-1): #ahora le damos la vuelta a la lista y escribimos en el txt
                for i in instrucciones[j]:
                    matriz_txt.write(i+"\n")
                np.savetxt(matriz_txt,matriz[j],fmt="%.0f")

            self.tamanhoplan=len(instrucciones)

        if not exito:
            print("Fracaso en la busqueda de solucion")
            sys.exit()


    def hay_pallet(self,estado,x,y):
        if estado.pallet_cargado==None: return False
        else:
            copia=copy.deepcopy(estado.pactual)
            copia.pop(estado.pallet_cargado)
            for pallet in copia:
                #print(pallet[0]==x, pallet[1]==y)
                if pallet[0]==x and pallet[1]==y: return True
            return False


    def expandir_nodo(self,estado):
        matrix_new=estado.matrix
        pactual_update=copy.deepcopy(estado.pactual)

        ins0=["nav.move(1)"]
        ins1=["nav.rotateRight()"]
        ins2=["nav.rotateLeft()"]
        ins3=["nav.downLift()"]
        ins4=["nav.upLift()"]
        

        if estado.robota==0 and (matrix_new[estado.robotf-1,estado.robotc]==0) and not self.hay_pallet(estado,estado.robotf-1,estado.robotc) : 
            matrix_new0=matrix_new.copy()
            matrix_new0[estado.robotf,estado.robotc]=0
            matrix_new0[estado.robotf-1,estado.robotc]=1
            pactual_update0=copy.deepcopy(pactual_update)
            if type(estado.pallet_cargado) == int:
                pactual_update0[estado.pallet_cargado][0]==int(np.where(matrix_new0==1)[0])
                pactual_update0[estado.pallet_cargado][1]=int(np.where(matrix_new0==1)[1])

            if type(estado.pallet_cargado) == int: coste=2
            else: coste=1
            if type(estado.pallet_cargado) == int: nuevo_estado=Estado(matrix_new0,0,estado,ins0,pactual=pactual_update0,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new0,0,estado,ins0,pactual=pactual_update0,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            
            del pactual_update

        if estado.robota==90 and (matrix_new[estado.robotf,estado.robotc+1]==0) and not self.hay_pallet(estado,estado.robotf,estado.robotc+1): 
            matrix_new1=matrix_new.copy() 
            matrix_new1[estado.robotf,estado.robotc]=0
            matrix_new1[estado.robotf,estado.robotc+1]=1

            pactual_update1=copy.deepcopy(pactual_update)
            if type(estado.pallet_cargado) == int:
                pactual_update1[estado.pallet_cargado][0]=int(np.where(matrix_new1==1)[0])
                pactual_update1[estado.pallet_cargado][1]=int(np.where(matrix_new1==1)[1])
            if type(estado.pallet_cargado) == int: coste=2
            else: coste=1
            if type(estado.pallet_cargado) == int: nuevo_estado=Estado(matrix_new1,90,estado,ins0,pactual=pactual_update1,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new1,90,estado,ins0,pactual=pactual_update1,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            del pactual_update1

        if estado.robota==180 and (matrix_new[estado.robotf+1,estado.robotc]==0) and not self.hay_pallet(estado,estado.robotf+1,estado.robotc): 
            matrix_new2=matrix_new.copy()
            matrix_new2[estado.robotf,estado.robotc]=0
            matrix_new2[estado.robotf+1,estado.robotc]=1

            pactual_update2=copy.deepcopy(pactual_update)
            if type(estado.pallet_cargado) == int:
                pactual_update2[estado.pallet_cargado][0]=int(np.where(matrix_new2==1)[0])
                pactual_update2[estado.pallet_cargado][1]=int(np.where(matrix_new2==1)[1])

            if type(estado.pallet_cargado) == int: coste=2
            else: coste=1
            #if estado.pallet_cargado and self.palletocupa3(estado):nuevo_estado=Estado(matrix_new0,180,estado,ins0,pactual=estado.pactual,pallet_cargado=estado.pallet_cargado,pallet_final=estado.pallet_final,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if type(estado.pallet_cargado) == int:nuevo_estado=Estado(matrix_new2,180,estado,ins0,pactual=pactual_update2,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new2,180,estado,ins0,pactual=pactual_update2,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            del pactual_update2

        if estado.robota==270 and (matrix_new[estado.robotf,estado.robotc-1]==0) and not self.hay_pallet(estado,estado.robotf,estado.robotc-1): 
            matrix_new3=matrix_new.copy()
            matrix_new3[estado.robotf,estado.robotc]=0
            matrix_new3[estado.robotf,estado.robotc-1]=1

            pactual_update3=copy.deepcopy(pactual_update)
            if type(estado.pallet_cargado) == int:
                pactual_update3[estado.pallet_cargado][0]=int(np.where(matrix_new3==1)[0])
                pactual_update3[estado.pallet_cargado][1]=int(np.where(matrix_new3==1)[1])

            if type(estado.pallet_cargado) == int: coste=2
            else: coste=1
            #if estado.pallet_cargado and self.palletocupa3(estado): nuevo_estado=Estado(matrix_new0,270,estado,ins0,pactual=estado.pactual,pallet_cargado=estado.pallet_cargado,pallet_final=estado.pallet_final,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if type(estado.pallet_cargado) == int: nuevo_estado=Estado(matrix_new3,270,estado,ins0,pactual=pactual_update3,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new3,270,estado,ins0,pactual=pactual_update3,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            del pactual_update3


        if True: #giro izquierda
            if type(estado.pallet_cargado) == int: coste=3
            else: coste=2
            angle=estado.robota
            pactual_update4=copy.deepcopy(estado.pactual)
            
            if type(estado.pallet_cargado) == int:
                if estado.pactual[estado.pallet_cargado][2]=="V": pactual_update4[estado.pallet_cargado][2]="H"
                elif estado.pactual[estado.pallet_cargado][2]=="H": pactual_update4[estado.pallet_cargado][2]="V"
            
            
            if type(estado.pallet_cargado) == int: nuevo_estado=Estado(matrix_new,(angle-90)%360,estado,ins2,pactual=pactual_update4,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new,(angle-90)%360,estado,ins2,pactual=pactual_update4,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            del pactual_update4


        if True: #giro derecha
            if type(estado.pallet_cargado) == int: coste=3
            else: coste=2
            angle=estado.robota
            pactual_update5=copy.deepcopy(estado.pactual)

            if type(estado.pallet_cargado) == int:
                if estado.pactual[estado.pallet_cargado][2]=="V": pactual_update5[estado.pallet_cargado][2]="H"
                elif estado.pactual[estado.pallet_cargado][2]=="H": pactual_update5[estado.pallet_cargado][2]="V"
        
            
            if type(estado.pallet_cargado) == int: nuevo_estado=Estado(matrix_new,(angle+90)%360,estado,ins1,pactual=pactual_update5,pallet_cargado=estado.pallet_cargado,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            if estado.pallet_cargado==None: nuevo_estado=Estado(matrix_new,(angle+90)%360,estado,ins1,pactual=pactual_update5,coste_accion=coste); self.anhadir_estado(nuevo_estado)
            del pactual_update5

        #vamos a generar estados de carga, para esto el pallet no puede estar en la pose final
        if estado.pallet_cargado==None and self.comprobar_cargar(estado):
            nuevo_estado=Estado(matrix_new,estado.robota,estado,ins4,pactual=estado.pactual,pallet_cargado=int(self.pallet_levantar(estado)),coste_accion=3)
            self.anhadir_estado(nuevo_estado)

        #generamos instrucciones de descarga
        #if type(estado.pallet_cargado) == int and estado.pactual[estado.pallet_cargado]==estado.pallet_obj[estado.pallet_cargado]:
        if type(estado.pallet_cargado) == int:
            nuevo_estado=Estado(matrix_new,estado.robota,estado,ins3,pallet_cargado=None,pactual=estado.pactual,coste_accion=3)
            #self.imprimir(nuevo_estado)
            self.anhadir_estado(nuevo_estado)

    def comprobar_cargar(self,estado):
        pose_robot=(estado.robotf,estado.robotc)
        for i,pallet in enumerate(estado.pactual):
            #comprobamos que el pallet esta encima del robot y que no esta en la posicion final donde hay que dejarlo
            if (pallet[0],pallet[1])==(estado.robotf,estado.robotc) and (pallet[0],pallet[1])!=(estado.pallet_obj[i][0],estado.pallet_obj[i][1]):
                return True
        return False


    def pallet_levantar(self,estado):
        pose_robot=(estado.robotf,estado.robotc)
        
        for i,pallet in enumerate(estado.pactual):
            #comprobamos que el pallet esta encima del robot y que no esta en la posicion final donde hay que dejarlo
            if (pallet[0],pallet[1])==(estado.robotf,estado.robotc):
                #print(pose_robot,pallet[0],pallet[1])
                return i

        
    def anhadir_estado(self,estado_nuevo):
        anhadir=True
        funcion_evaluacion=estado_nuevo.f
        #comprobar que no este ni en abierta ni en cerrada
        for i in range(len(self.cerrada)): #comprobamos que no este en cerrada
            if estado_nuevo==self.cerrada[i]: anhadir=False; return
        if anhadir:   
            for i in range(len(self.abierta)): #comprobamos que no este en abierta
                if estado_nuevo==self.abierta[i]: 
                    if (self.abierta[i]).f>funcion_evaluacion: #si esta en abierta miramos si tiene mejor valor de la funcion de evaluacion
                        self.abierta.remove(self.abierta[i]) #eliminamos el antiguo de abierta
                        break
                    else:
                        anhadir=False; return

        if anhadir:       
            exito=False #si ya anadimos no hace falta anadir al final de la lista
            for i in range (len(self.abierta)):
                if funcion_evaluacion<=(self.abierta[i].f):
                    self.abierta.insert(i,estado_nuevo)
                    exito=True
                    break #anadimos y salimos del bucle
            if not exito: self.abierta.append(estado_nuevo); return #si no tuvimos exito anadimos al final

    """def anhadir_estado(self,estado_nuevo):
        funcion_evaluacion=estado_nuevo.f
        for i in range (len(self.abierta)):
            if funcion_evaluacion<=(self.abierta[i].f):
                self.abierta.insert(i,estado_nuevo)
                return #anadimos y salimos del bucle
        self.abierta.append(estado_nuevo)"""


    def estado_inicial_final(self):
        """Funcion para localizar objetos en una matriz representativa de un mapa.
        -Localizamos el robot como un 1
        -Localizamos obtaculos/paredes/fuera de mapa como 9"""
        import sys
        #Las creo como variables globales para usarlas en heuristica
        global pose_inicio, pinicio, pfinal

        try:
            argumentos=sys.argv
            pinicio=[]
            pfinal=[]
            argumentos.pop(0)
            while argumentos!=[]:
                pallet_in=(int(argumentos[0][0]),int(argumentos[0][2]))
                argumentos.pop(0)
                pallet_in_orientacion=argumentos[0]
                argumentos.pop(0)
                pallet_fin=(int(argumentos[0][0]),int(argumentos[0][2]))
                argumentos.pop(0)
                pallet_fin_orientacion=argumentos[0]
                argumentos.pop(0)
                pinicio.append([pallet_in[0],pallet_in[1],pallet_in_orientacion])
                pfinal.append([pallet_fin[0],pallet_fin[1],pallet_fin_orientacion])
        except:
            print ("Se han introducido mal los argumentos, para que sean correctos tienen que tener la siguiente forma")
            print(">> python varios_pallets.py 3,1 V 1,1 V 3,4 V 1,4 V")   
            print(">> python script.py pose_inicial orien_inicial pose_final orien_final")  
            sys.exit()

        #matriz                     
        matrix=np.array(((9,9,9,9,9,9),
                        (9,0,0,0,0,9),
                        (9,0,0,0,0,9),
                        (9,0,0,1,0,9),
                        (9,9,9,9,9,9)))

        """matrix=np.array(((9,9,9,9),
                        (9,0,0,9),
                        (9,0,1,9),
                        (9,9,9,9)))"""

        #calculamos donde esta el robot en el inicio
        pose_inicio=[int(np.where(matrix==1)[0]),int(np.where(matrix==1)[1])]

        #definimos caracteristicas para de los pallets
        """pinicio=[[3,1,"H"],[3,4,"H"]]
        pfinal=[[1,1,"H"],[1,4,"H"]]"""


        #creamos estado inicial
        A=Estado(matrix,0,None,instrucciones=None,pactual=pinicio,pallet_final=pfinal)
        
        #creamos estado final
        B=Estado(matrix,0,None,instrucciones=None,pactual=pfinal,pallet_final=pfinal)
        
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


p = Practica1()


"""if __name__ == '__main__':
    try:
        p = Practica1()
    except (RuntimeError, TypeError, NameError) as err:
        rospy.loginfo("Practica2 terminated: ", err)"""