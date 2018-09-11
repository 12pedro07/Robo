#!/usr/bin/env python
import numpy
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import smach
import smach_ros


class ObjectsOrganizer(smach.State):

    def __init__(self, outcomes=['success', 'fail']):
        smach.State.__init__(self, outcomes=['success', 'fail'], output_keys=['left2right'])
        self.before = [0]
        self.final_list = []
        self.odata = []
        rospy.Subscriber("/objects", Float32MultiArray, self.callback)

    def callback(self, data):

        self.odata = data.data

    def execute(self, userdata):
        c = 0
        while c < 30:
            rospy.sleep(0.5)
            obj_dict = {
                # O range dos objetos, iniciando por 1 deve conter o numero de imagens a serem utilizadas para cada objeto
                'calculadora': list(range(1, 5)),  # imagens 1 e 2
                'baralho': list(range(5, 6)),  # apenas imagem 3
                'agua': list(range(6, 11)),  # imagem de 4 a 10
                #'coca'                 : range(1,11),        # 10 imagens
                #'calculadora'         : range(11,21),        # 10 imagens
                #'copo'                 : range(21,31),        # 10 imagens
                #'...'                 : range (31,...)    # ...
            }
            obj_pos_dict = {}  # este dicionario armazena os dados de posicao dos itens com o nome igual ao do objeto + _pos
            if len(self.odata) != 0:
                for x in range(len(self.odata) / 12):  # x variando de acordo com o numero de objetos encontrados
                    y = x * 12  # y eh um auxiliar para encontrar o ID do objeto dentro de data.data.
                    rospy.loginfo('/object_' + str(int(self.odata[y])))
                    aux = 0  # aux utilizado caso o objeto nao tenha sido encontrado dentro den obj_dict.
                    for item in obj_dict:  # "item" recebendo as strings com os nomes de cada item do dicionario, um por vez.
                        for n in obj_dict[item]:  # para cada item do dicionario, n recebe os valores dos 'range()' armazenados neste.
                            if n == self.odata[y]:  # caso n esteja dentro do 'range()' do item.
                                rospy.loginfo("object found -> " + str(item))  # imprime o item do dicionario relativo ao objeto.
                                pos_coord = x + 9  # pos_coord guarda o valor da coordenada onde esta a posicao do objeto no tuple
                                obj_pos_dict[str(item)] = self.odata[pos_coord]  # guardando o dado da posicao no dicionario
                                rospy.loginfo('x pos = ' + str(self.odata[pos_coord]))
                                rospy.loginfo("-" * 24)
                                aux = 1
                                #rospy.loginfo('item  = ' + str(item))
                                #rospy.loginfo('n     = ' + str(n))
                                #rospy.loginfo('x     = ' + str(x))
                                #print self.odata
                                #print 'pos_coord = ' + str(self.odata[pos_coord])
                            #print "item "+ str(item) +", aux = " +str(aux)            # todos estes comentados sao para testes
                    if aux == 0:
                        rospy.loginfo("object unknown")
                        rospy.loginfo("-" * 24)
                obj_esq = min(obj_pos_dict, key=obj_pos_dict.get)  # menor valor do dicionario, que equivale ao objeto mais a esquerda
                print(obj_pos_dict)
                rospy.loginfo('objeto mais a esquerda = ' + obj_esq)
                # escolha z conforme deseja a variavel de saida (padrao z = 2 - apenas os nomes)
                z = 2
                if z == 1:
                    # only values:
                    sorted_list = sorted(obj_pos_dict.values())
                elif z == 2:
                    # only names:
                    sorted_list = sorted(obj_pos_dict, key=obj_pos_dict.get)
                else:
                    # both:
                    sorted_list = sorted(list(obj_pos_dict.items()), key=lambda x: x[1])
                atual = len(sorted_list)
                print("before = " + str(self.before))
                print("objetos da esquerda para a direita : %s" % self.final_list)
                print("atual = " + str(atual))
                if atual > self.before[0]:
                    self.final_list = sorted_list
                    atual = len(sorted_list)
                self.before[0] = atual
                print("-------------------- // --------------------")
            else:
                rospy.loginfo('no objects detected')
            c += 1
        if self.final_list != []:
            userdata.left2right = self.final_list
            return 'success'
        else:
            return 'fail'
