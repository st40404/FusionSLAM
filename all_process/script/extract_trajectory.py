#!/usr/bin/env python
import os,glob
import matplotlib.pyplot as plt
import yaml
import math

class Trajectory():
    def __init__(self, path, file_name, my_list):
        # self.log_path = path + "/Log/" + file_name
        # self.save_path = path + "/Generate_Graph/" + file_name
        self.log_path = path + "/Trajectory_log" 
        self.save_path = path + "/Graph"
        self.res_path = path + "/Res_"
        self.weight_path = path + "/Weight_"
        self.mse_path = path + "/MSE_"
        self.z_path = path + "/z_"
        self.show_list = my_list

        self.point_plicp = []
        self.point_orb =   []

        self.point_ukf_plicp = []
        self.point_ukf_orb =   []
        
        self.point_real = []
        self.point_our_method = []

        self.point_residual_orb = []
        self.point_residual_plicp = []

        self.weight_orb = []
        self.weight_plicp = []

        self.point_half_weight = []

        self.mse_accu_orb = []
        self.mse_accu_plicp = []

        self.mse_curr_orb = []
        self.mse_curr_plicp = []

        self.hypo_orb = []
        self.hypo_plicp = []

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

        ######### save Half Weight point ###########
        self.point_half_weight.append([])
        self.point_half_weight.append([])
        self.point_half_weight.append([])
        for i in range(0, len(_data['Trajectory']['Half_w'])/3):
            self.point_half_weight[0].append( _data['Trajectory']['Half_w'][3*i] )
            self.point_half_weight[1].append( _data['Trajectory']['Half_w'][3*i+1] )
            self.point_half_weight[2].append( _data['Trajectory']['Half_w'][3*i+2] )

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

        ######### save weight of orb ###########
        # self.weight_orb.append([])
        # self.weight_orb.append([])
        # self.weight_orb.append([])
        # for i in range(0, len(_data['Weight_ORB']['x'])):
        #     self.weight_orb[0].append( _data['Weight_ORB']['x'][i] )
        #     self.weight_orb[1].append( _data['Weight_ORB']['y'][i] )
        #     self.weight_orb[2].append( _data['Residual']['time'][i] )

        # ######### save weight of plicp ###########
        # self.weight_plicp.append([])
        # self.weight_plicp.append([])
        # self.weight_plicp.append([])
        # for i in range(0, len(_data['Weight_PLICP']['x'])):
        #     self.weight_plicp[0].append( _data['Weight_PLICP']['x'][i] )
        #     self.weight_plicp[1].append( _data['Weight_PLICP']['y'][i] )
        #     self.weight_plicp[2].append( _data['Residual']['time'][i] )


        # ######### save z of hypothesis of orb ###########
        # self.hypo_orb.append([])
        # self.hypo_orb.append([])
        # self.hypo_orb.append([])
        # for i in range(0, len(_data['Hypothesis']['ORB_x'])):
        #     self.hypo_orb[0].append( _data['Hypothesis']['ORB_x'][i])
        #     self.hypo_orb[1].append( _data['Hypothesis']['ORB_y'][i])
        #     self.hypo_orb[2].append( _data['Residual']['time'][i] )


        # ######### save z of hypothesis of plicp ###########
        # self.hypo_plicp.append([])
        # self.hypo_plicp.append([])
        # self.hypo_plicp.append([])
        # for i in range(0, len(_data['Hypothesis']['PLICP_x'])):
        #     self.hypo_plicp[0].append( _data['Hypothesis']['PLICP_x'][i])
        #     self.hypo_plicp[1].append( _data['Hypothesis']['PLICP_y'][i])
        #     self.hypo_plicp[2].append( _data['Residual']['time'][i] )



    def SaveTrajecAll(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                # self.DrawAll(self.point_plicp, "mediumpurple", 'PLICP')
                self.DrawAll(self.point_plicp, "blue", 'PLICP')
            elif (i == 'ORB'):
                self.DrawAll(self.point_orb, "red", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawAll(self.point_ukf_plicp, "blueviolet", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawAll(self.point_ukf_orb, "blue", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawAll(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawAll(self.point_our_method, "green", 'Our_method')
            elif (i == 'Half_w'):
                self.DrawAll(self.point_half_weight, "pink", 'half_weight')

        plt.title( 'Trajectory')
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        # plt.xlim(-800, 800)
        # plt.ylim(-800, 800)
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_all")
        plt.clf()

    def SaveTrajecX(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                # self.DrawX(self.point_plicp, "mediumpurple", 'PLICP')
                self.DrawX(self.point_plicp, "blue", 'PLICP')
            elif (i == 'ORB'):
                self.DrawX(self.point_orb, "red", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawX(self.point_ukf_plicp, "blueviolet", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawX(self.point_ukf_orb, "blue", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawX(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawX(self.point_our_method, "green", 'Our_method')
            elif (i == 'Half_w'):
                self.DrawX(self.point_half_weight, "pink", 'half_weight')

        plt.title('Trajectory of X Axis')
        plt.xlabel('Time (s)')
        plt.ylabel('X (cm)')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_x")
        plt.clf()

    def SaveTrajecY(self):
        for i in self.show_list:
            if (i == 'PLICP'):
                # self.DrawY(self.point_plicp, "blueviolet", 'PLICP')
                self.DrawY(self.point_plicp, "blue", 'PLICP')
            elif (i == 'ORB'):
                self.DrawY(self.point_orb, "red", 'ORB')
            elif (i == 'UKF_PLICP'):
                self.DrawY(self.point_ukf_plicp, "mediumpurple", 'UKF_PLICP')
            elif (i == 'UKF_ORB'):
                self.DrawY(self.point_ukf_orb, "blue", 'UKF_ORB')
            elif (i == 'real'):
                self.DrawY(self.point_real, "black", 'ground_trust')
            elif (i == 'Our_method'):
                self.DrawY(self.point_our_method, "green", 'Our_method')
            elif (i == 'Half_w'):
                self.DrawY(self.point_half_weight, "pink", 'half_weight')

        plt.title('Trajectory of Y Axis')
        plt.xlabel('Time (s)')
        plt.ylabel('Y (cm)')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.save_path + "_y")
        plt.clf()

    def SaveResidual(self):
        # for i in self.show_list:
        #     if (i == 'UKF_PLICP'):
        #         self.DrawRes(self.point_residual_plicp[0], self.point_residual_plicp[2], 'PLICP', "X")
        #         self.DrawRes(self.point_residual_plicp[1], self.point_residual_plicp[2], 'PLICP', "Y")
        #     elif (i == 'UKF_ORB'):
        #         self.DrawRes(self.point_residual_orb[0], self.point_residual_orb[2], 'ORB', "X")
        #         self.DrawRes(self.point_residual_orb[1], self.point_residual_orb[2], 'ORB', "Y")
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
        plt.title(_ylabel + ' Axis Residual of ' + _label)
        plt.plot(time, x_lab, color = "black", linewidth ='1', label = _label)
        plt.xlabel('Time (s)')
        plt.ylabel(_ylabel + ' (cm)')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.res_path + _label + "_" + _ylabel)
        plt.clf()

    def SaveResidual_(self):
        res_x = []
        res_y = []
        res_time = []
        for i in range (0, len(self.point_residual_plicp[0])):
            res_x.append(self.point_residual_orb[0][i] - self.point_residual_plicp[0][i])
            res_y.append(self.point_residual_orb[1][i] - self.point_residual_plicp[1][i])
            res_time.append(self.point_residual_orb[2][i])

        plt.title("x_res")
        plt.plot(res_time, res_x, color = "black", linewidth ='1')
        plt.xlabel('Time (s)')
        plt.ylabel( 'x' + ' (cm)')
        plt.savefig(self.res_path + "diff_x")
        plt.clf()

        plt.title("y_res")
        plt.plot(res_time, res_y, color = "black", linewidth ='1')
        plt.xlabel('Time (s)')
        plt.ylabel( 'y' + ' (cm)')
        plt.savefig(self.res_path + "diff_y")
        plt.clf()


    def SaveWeight(self):
        self.DrawWeight(self.weight_orb[2], self.weight_orb[0], self.weight_plicp[2], self.weight_plicp[0], "ORB", "PLICP", "X")
        self.DrawWeight(self.weight_orb[2], self.weight_orb[1], self.weight_plicp[2], self.weight_plicp[1], "ORB", "PLICP", "Y")


    def DrawWeight(self, x1, y1, x2, y2, method_1, method_2, _ylabel):
        plt.plot(x1, y1, color = 'red', linewidth ='1', label = method_1)
        plt.plot(x2, y2, color = 'blue', linewidth ='1', label = method_2)
        plt.title( _ylabel + ' Axis Weight')
        plt.xlabel('Time (s)')
        plt.ylabel(_ylabel + " (%)")
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.weight_path + "_" + _ylabel)
        plt.clf()

    def SaveAllMSE(self):
        self.mse_accu_orb.append([])
        self.mse_accu_orb.append([])
        self.mse_accu_orb.append([])
        self.mse_accu_plicp.append([])
        self.mse_accu_plicp.append([])
        self.mse_accu_plicp.append([])

        # compute PLICP each time's accumulate MSE
        self.ComeputeAllMSE(self.point_plicp, self.point_real, self.mse_accu_plicp)

        # compute ORB each time's accumulate MSE
        self.ComeputeAllMSE(self.point_orb, self.point_real, self.mse_accu_orb)

        self.DrawMSE(self.mse_accu_orb[2], self.mse_accu_orb[0], self.mse_accu_plicp[2], self.mse_accu_plicp[0], "ORB", "PLICP", "X")
        self.DrawMSE(self.mse_accu_orb[2], self.mse_accu_orb[1], self.mse_accu_plicp[2], self.mse_accu_plicp[1], "ORB", "PLICP", "Y")

    def SaveCurMSE(self):
        self.mse_curr_orb.append([])
        self.mse_curr_orb.append([])
        self.mse_curr_orb.append([])
        self.mse_curr_plicp.append([])
        self.mse_curr_plicp.append([])
        self.mse_curr_plicp.append([])
        # compute PLICP each time's MSE
        self.ComeputeCurMSE(self.point_plicp, self.point_real, self.mse_curr_plicp)

        # compute ORB each time's MSE
        self.ComeputeCurMSE(self.point_orb, self.point_real, self.mse_curr_orb)

        self.DrawMSE(self.mse_curr_orb[2], self.mse_curr_orb[0], self.mse_curr_plicp[2], self.mse_curr_plicp[0], "ORB", "PLICP", "X")
        self.DrawMSE(self.mse_curr_orb[2], self.mse_curr_orb[1], self.mse_curr_plicp[2], self.mse_curr_plicp[1], "ORB", "PLICP", "Y")

    def SaveHypoZ(self):
        self.DrawZ(self.hypo_orb[2], self.hypo_orb[0], "ORB_x")
        self.DrawZ(self.hypo_orb[2], self.hypo_orb[1], "ORB_y")
        
        self.DrawZ(self.hypo_plicp[2], self.hypo_plicp[0], "PLICP_x")
        self.DrawZ(self.hypo_plicp[2], self.hypo_plicp[1], "PLICP_y")

    def DrawZ(self, x, y, _ylabel):
        plt.plot(x, y, color = 'red', linewidth ='1')
        plt.title('Hypothesis z of ' + _ylabel)
        plt.xlabel('Time (s)')
        plt.ylabel('z')
        plt.savefig(self.z_path + "_" + _ylabel)
        plt.clf()

    def ComeputeCurMSE(self, _target, _refer, _contain):
        for i in range(0, len(_target[0])):
            _contain[0].append(((_refer[0][i] - _target[0][i])/100) ** 2)
            _contain[1].append(((_refer[1][i] - _target[1][i])/100) ** 2)
            _contain[2].append(_target[2][i])

    def ComeputeAllMSE(self, _target, _refer, _contain):
        mse_target_x  = 0.0
        mse_target_y  = 0.0

        for i in range(0, len(_target[0])):
            mse_target_x += ((_refer[0][i] - _target[0][i])/100) ** 2
            mse_target_y += ((_refer[1][i] - _target[1][i])/100) ** 2

            _contain[0].append(mse_target_x/(i+1))
            _contain[1].append(mse_target_y/(i+1))
            _contain[2].append(_target[2][i])

    def DrawMSE(self, x1, y1, x2, y2, method_1, method_2, _ylabel):
        plt.plot(x1, y1, color = 'blue', linewidth ='1', label = method_1)
        plt.plot(x2, y2, color = 'red', linewidth ='1', label = method_2)
        plt.title( _ylabel + ' Axis MSE (m)')
        plt.xlabel('Time (s)')
        plt.ylabel(_ylabel + '_MSE(m)')
        plt.legend(shadow=True, facecolor='ivory')
        plt.savefig(self.mse_path + "_" + _ylabel)
        plt.clf()


if __name__=='__main__':
    # path = '/home/ron/work/src/all_process/data/Trajectory'
    # file_name = '2023_05_26_12:04:44'

    name = 'orb'
    path = '/home/ron/work/src/all_process/data/realistic_test/' + name
    file_name = ''

    all_list = ['PLICP', 'ORB', 'UKF_PLICP', 'UKF_ORB', 'real', 'Our_method', 'Half_w']
    show_list = ['PLICP', 'ORB', 'real','Our_method']
    # show_list = ['PLICP', 'ORB']
    # show_list = ['PLICP', 'UKF_PLICP']
    show_list = ['ORB', 'UKF_ORB']

    # show_list = ['PLICP', 'ORB', 'UKF_PLICP', 'UKF_ORB', 'real', 'Our_method', 'Half_w']
    # show_list = ['PLICP', 'ORB', 'real']

    Tj_ = Trajectory(path, file_name, show_list)
    Tj_.LoadTrajec()
    # Choose what graph you want
    Tj_.SaveTrajecAll()
    Tj_.SaveTrajecX()
    Tj_.SaveTrajecY()
    Tj_.SaveResidual()
    Tj_.SaveResidual_()
    Tj_.SaveWeight()
    # Tj_.SaveAllMSE()
    Tj_.SaveCurMSE()
    Tj_.SaveHypoZ()

