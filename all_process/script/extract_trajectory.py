#!/usr/bin/env python
import os,glob
import matplotlib.pyplot as plt
import yaml


class Trajectory():
    def __init__(self, path, file_name, my_list):
        # self.log_path = path + "/Log/" + file_name
        # self.save_path = path + "/Generate_Graph/" + file_name
        self.log_path = path + "/Trajectory_log" 
        self.save_path = path + "/Graph"
        self.res_path = path + "/Res_"
        self.show_list = my_list

        self.point_plicp = []
        self.point_orb =   []

        self.point_ukf_plicp = []
        self.point_ukf_orb =   []
        
        self.point_real = []
        self.point_our_method =   []

        self.point_residual_orb = []
        self.point_residual_plicp =   []

# Generate_Graph

    def LoadTrajec(self):
        with open( self.log_path + '.yaml', 'r') as f:
            _data = yaml.load(f)
            
        ######### save PLICP point ###########
        self.point_plicp.append([])
        self.point_plicp.append([])
        self.point_plicp.append([])
        for i in range(0, len(_data['Trajectory']['PLICP'])/3):
            self.point_plicp[0].append( _data['Trajectory']['PLICP'][3*i] )
            self.point_plicp[1].append( _data['Trajectory']['PLICP'][3*i+1] )
            self.point_plicp[2].append( _data['Trajectory']['PLICP'][3*i+2] )

        ######### save ORB point ###########
        self.point_orb.append([])
        self.point_orb.append([])
        self.point_orb.append([])
        for i in range(0, len(_data['Trajectory']['ORB'])/3):
            # if (i%2 == 1):
            self.point_orb[0].append( _data['Trajectory']['ORB'][3*i] )
            self.point_orb[1].append( _data['Trajectory']['ORB'][3*i+1] )
            self.point_orb[2].append( _data['Trajectory']['ORB'][3*i+2] )

        ######### save UKF_PLICP point ###########
        self.point_ukf_plicp.append([])
        self.point_ukf_plicp.append([])
        self.point_ukf_plicp.append([])
        for i in range(0, len(_data['Trajectory']['UKF_PLICP'])/3):
            self.point_ukf_plicp[0].append( _data['Trajectory']['UKF_PLICP'][3*i] )
            self.point_ukf_plicp[1].append( _data['Trajectory']['UKF_PLICP'][3*i+1] )
            self.point_ukf_plicp[2].append( _data['Trajectory']['UKF_PLICP'][3*i+2] )

        ######### save UKF_ORB point ###########
        self.point_ukf_orb.append([])
        self.point_ukf_orb.append([])
        self.point_ukf_orb.append([])
        for i in range(0, len(_data['Trajectory']['UKF_ORB'])/3):
            self.point_ukf_orb[0].append( _data['Trajectory']['UKF_ORB'][3*i] )
            self.point_ukf_orb[1].append( _data['Trajectory']['UKF_ORB'][3*i+1] )
            self.point_ukf_orb[2].append( _data['Trajectory']['UKF_ORB'][3*i+2] )

        ######### save real point ###########
        self.point_real.append([])
        self.point_real.append([])
        self.point_real.append([])
        for i in range(0, len(_data['Trajectory']['real'])/3):
            self.point_real[0].append( _data['Trajectory']['real'][3*i] )
            self.point_real[1].append( _data['Trajectory']['real'][3*i+1] )
            self.point_real[2].append( _data['Trajectory']['real'][3*i+2] )

        ######### save Our_method point ###########
        self.point_our_method.append([])
        self.point_our_method.append([])
        self.point_our_method.append([])
        for i in range(0, len(_data['Trajectory']['Our_method'])/3):
            self.point_our_method[0].append( _data['Trajectory']['Our_method'][3*i] )
            self.point_our_method[1].append( _data['Trajectory']['Our_method'][3*i+1] )
            self.point_our_method[2].append( _data['Trajectory']['Our_method'][3*i+2] )

        ######### save ORBSLAM2 Residual of x point and time ###########
        self.point_residual_orb.append([])
        self.point_residual_orb.append([])
        self.point_residual_orb.append([])
        for i in range(0, len(_data['Residual_ORB']['x'])):
            self.point_residual_orb[0].append( _data['Residual_ORB']['x'][i] )
            self.point_residual_orb[1].append( _data['Residual_ORB']['y'][i] )
            self.point_residual_orb[2].append( _data['Residual']['time'][i] )

        ######### save PLICP Residual of x point and time ###########
        self.point_residual_plicp.append([])
        self.point_residual_plicp.append([])
        self.point_residual_plicp.append([])
        for i in range(0, len(_data['Residual_PLICP']['x'])):
            self.point_residual_plicp[0].append( _data['Residual_PLICP']['x'][i] )
            self.point_residual_plicp[1].append( _data['Residual_PLICP']['y'][i] )
            self.point_residual_plicp[2].append( _data['Residual']['time'][i] )



    def SaveTrajecAll(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                self.DrawAll(self.point_plicp, "mediumpurple", 'PLICP')
            elif (i == 'ORB'):
                self.DrawAll(self.point_orb, "limegreen", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawAll(self.point_ukf_plicp, "blueviolet", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawAll(self.point_ukf_orb, "lime", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawAll(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawAll(self.point_our_method, "red", 'Our_method')

        plt.xlabel('Y')
        plt.ylabel('X')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_all")
        plt.clf()

    def SaveTrajecX(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                self.DrawX(self.point_plicp, "mediumpurple", 'PLICP')
            elif (i == 'ORB'):
                self.DrawX(self.point_orb, "limegreen", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawX(self.point_ukf_plicp, "blueviolet", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawX(self.point_ukf_orb, "lime", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawX(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawX(self.point_our_method, "red", 'Our_method')

        plt.xlabel('time')
        plt.ylabel('X')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_x")
        plt.clf()

    def SaveTrajecY(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                self.DrawY(self.point_plicp, "blueviolet", 'PLICP')
            elif (i == 'ORB'):
                self.DrawY(self.point_orb, "lime", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawY(self.point_ukf_plicp, "mediumpurple", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawY(self.point_ukf_orb, "limegreen", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawY(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawY(self.point_our_method, "red", 'Our_method')

        plt.xlabel('time')
        plt.ylabel('Y')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_y")
        plt.clf()

    def SaveResidual(self):
        for i in self.show_list:
            if (i == 'UKF_PLICP'):
                self.DrawRes(self.point_residual_plicp[0], self.point_residual_plicp[2], 'PLICP', "X")
                self.DrawRes(self.point_residual_plicp[1], self.point_residual_plicp[2], 'PLICP', "Y")
            elif (i == 'UKF_ORB'):
                self.DrawRes(self.point_residual_orb[0], self.point_residual_orb[2], 'ORB', "X")
                self.DrawRes(self.point_residual_orb[1], self.point_residual_orb[2], 'ORB', "Y")
            else:
                self.DrawRes(self.point_residual_plicp[0], self.point_residual_plicp[2], 'PLICP', "X")
                self.DrawRes(self.point_residual_plicp[1], self.point_residual_plicp[2], 'PLICP', "Y")
                self.DrawRes(self.point_residual_orb[0], self.point_residual_orb[2], 'ORB', "X")
                self.DrawRes(self.point_residual_orb[1], self.point_residual_orb[2], 'ORB', "Y")

    def DrawAll(self, _data, _color, _label):
        plt.plot(_data[0], _data[1], color = _color, linewidth ='1', label = _label)

    def DrawX(self, _data, _color, _label):
        plt.plot(_data[2], _data[0], color = _color, linewidth ='1', label = _label)

    def DrawY(self, _data, _color, _label):
        plt.plot(_data[2], _data[1], color = _color, linewidth ='1', label = _label)

    def DrawRes(self, x_lab, time, _label, _ylabel):
        plt.plot(time, x_lab, color = "black", linewidth ='1', label = _label)
        plt.xlabel('time')
        plt.ylabel(_ylabel)
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.res_path + _label + "_" + _ylabel)
        plt.clf()

if __name__=='__main__':
    # path = '/home/ron/work/src/all_process/data/Trajectory'
    # file_name = '2023_05_26_12:04:44'

    name = '2023_05_28_15:33:55'
    path = '/home/ron/work/src/all_process/data/realistic_test/' + name
    file_name = ''

    all_list = ['PLICP', 'ORB', 'UKF_PLICP', 'UKF_ORB', 'real', 'Our_method']
    # show_list = ['PLICP', 'ORB', 'real','Our_method']
    # show_list = ['PLICP', 'ORB']
    show_list = ['PLICP', 'ORB', 'real']
    # show_list = ['ORB']
    # show_list = ['PLICP', 'ORB', 'UKF_PLICP', 'UKF_ORB', 'real','Our_method']

    Tj_ = Trajectory(path, file_name, show_list)
    Tj_.LoadTrajec()
    # Choose what graph you want
    Tj_.SaveTrajecAll()
    Tj_.SaveTrajecX()
    Tj_.SaveTrajecY()
    Tj_.SaveResidual()
