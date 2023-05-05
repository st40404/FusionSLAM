#!/usr/bin/env python
import os,glob
import matplotlib.pyplot as plt



class Result():
    def __init__(self, path):
        # output of the log (avg, MSEx. MSEy, MSEs)
        self.output = 4
        self.path = path
        self.file_list = []
        # It save every line of list, but first member is header name
        # such as self.param_list[Line][type value]
        self.param_list = []
        # It save every type name
        # such as self.header = ["Feature", "scale", "level"]
        self.header = []

        self.best_avg = []
        self.best_MSE_x = []
        self.best_MSE_y = []
        self.best_MSE_s = []

        # It save every size and param type
        # such as self.param_size[Features][500]
        self.param_size = []
        # It save every param of param type, , but first member is header type
        # such as self.param_statistics[Features][500][line num of self.param_list]
        # such as self.param_statistics[0] = 500, self.param_statistics[1] = 1500
        self.param_statistics = []
        # save every average of result
        self.best_param = []

        self.computeBestResult()
        self.computeBestParam()
        self.drawGraph()


    def computeBestResult(self):
        self.getTxtList()
        self.getLineTitle()
        self.getParamList()
        self.bestResult()

    def getTxtList(self):
        for filename in glob.glob(os.path.join(self.path , '*.txt')):
            self.file_list.append(filename)

    def getLineTitle(self):
        with open(self.file_list[0]) as f:
            first_line = f.readline()
            param = ""
            save_param = True
            count_minus = 0

            for char in first_line:
                if ( char.find(':') != -1 ):
                    self.header.append(param)
                    param = ""
                    save_param = False
                elif (char.find(':') != 1 & save_param == True):
                    param = param + char
                elif (char.isspace()):
                    save_param = True
                elif (char.find('-') != -1):
                    count_minus += 1
                    if (count_minus == 3):
                        save_param = True
        self.param_list.append(self.header)

    def getParamList(self):
        for i in self.file_list:
            with open(i) as f:
                lines = f.readlines()

                for word in lines:

                    line_list = word.replace(("---"), ':')
                    line_list = line_list.replace(("\n"), ":")
                    line_list = line_list.replace((" "), ':').split(":")
                    
                    insert_list = []
                    for i in range(0,len(self.header)*2):
                        if ( i==0 | i%2==0):
                            pass
                        elif (i%2==1):
                            insert_list.append(line_list[i])

                    self.param_list.append(insert_list)

    def bestResult(self):
        for line in range(1, len(self.param_list)):
            if (line == 1):
                self.best_avg.append(self.param_list[line][len(self.header)-4])
                self.best_avg.append(line)
                self.best_MSE_x.append(self.param_list[line][len(self.header)-3])
                self.best_MSE_x.append(line)
                self.best_MSE_y.append(self.param_list[line][len(self.header)-2])
                self.best_MSE_y.append(line)
                self.best_MSE_s.append(self.param_list[line][len(self.header)-1])
                self.best_MSE_s.append(line)
            else:
                if ( self.best_avg[0] > self.param_list[line][len(self.header)-4]):
                    self.best_avg[0] = self.param_list[line][len(self.header)-4]
                    self.best_avg[1] = line

                if ( self.best_MSE_x[0] > self.param_list[line][len(self.header)-3]):
                    self.best_MSE_x[0] = self.param_list[line][len(self.header)-3]
                    self.best_MSE_x[1] = line

                if ( self.best_MSE_y[0] > self.param_list[line][len(self.header)-2]):
                    self.best_MSE_y[0] = self.param_list[line][len(self.header)-2]
                    self.best_MSE_y[1] = line

                if ( self.best_MSE_s[0] > self.param_list[line][len(self.header)-1]):
                    self.best_MSE_s[0] = self.param_list[line][len(self.header)-1]
                    self.best_MSE_s[1] = line

        self.saveParam(self.best_avg)
        self.saveParam(self.best_MSE_x)
        self.saveParam(self.best_MSE_y)
        self.saveParam(self.best_MSE_s)

    def saveParam(self, best):
        best_row = best[1]

        for i in range (0, len(self.header)- self.output):
            try:
                best[i+1] = self.param_list[best_row][i]
            except:
                best.append(self.param_list[best_row][i])
        return best

    def computeBestParam(self):
        self.getAllParamSize()
        self.getAllParam()
        self.bestParamList()

    def getAllParamSize(self):
        for i in range(0, len(self.header)- self.output):
            flag_param_list = []
            flag_count = 0

            for j in range(1, len(self.param_list)):
                if (j == 1):
                    flag_param_list.append(self.param_list[j][i])
                else:
                    for k in range(0, len(flag_param_list) ):
                        if (flag_param_list[k] == self.param_list[j][i]):
                            flag_count = 0
                            break
                        elif ( flag_param_list[k] != self.param_list[j][i] ):
                            flag_count += 1
                    if (flag_count == len(flag_param_list)):
                        flag_param_list.append(self.param_list[j][i])
                        flag_count = 0
            self.param_size.append(flag_param_list)

    def getAllParam(self):
        flag_statistics = []

        ## get empty list of three dimantion matrix (param -> param_type -> param_position_in_param_list)
        for params in range(0, len(self.header)- self.output):
            for param in range (0, len(self.param_size[params])):
                flag_statistics.append([])
            self.param_statistics.append(flag_statistics)
            flag_statistics = []

        for params_list in range(1, len(self.param_list)):
            for params in range(0, len(self.header)- self.output):
                for param in range (0, len(self.param_size[params])):
                    if (self.param_size[params][param] == self.param_list[params_list][params]):
                        if (len(self.param_statistics[params][param]) == 0):
                            self.param_statistics[params][param].append(self.param_size[params][param])
                        self.param_statistics[params][param].append(params_list)
                        break

    def bestParamList(self):
        for params in range(0, len(self.param_statistics)):
            for param in range(0, len(self.param_statistics[params])):
                flag_avg_sum = 0.0
                flag_MSEx_sum = 0.0
                flag_MSEy_sum = 0.0
                flag_MSEs_sum = 0.0

                for data in range(0, len(self.param_statistics[params][param])):
                    if (data == 0):
                        print ("==================================================")
                        print ("=========  Average {} of type {}  =========").format(self.header[params], self.param_statistics[params][param][data])
                    else:
                        flag_avg_sum  += float(self.param_list[self.param_statistics[params][param][data]][len(self.header)-4])
                        flag_MSEx_sum += float(self.param_list[self.param_statistics[params][param][data]][len(self.header)-3])
                        flag_MSEy_sum += float(self.param_list[self.param_statistics[params][param][data]][len(self.header)-2])
                        flag_MSEs_sum += float(self.param_list[self.param_statistics[params][param][data]][len(self.header)-1])

                print ("avg  :  {}").format(flag_avg_sum /float(len(self.param_statistics[params][param])))
                print ("MSEx :  {}").format(flag_MSEx_sum/float(len(self.param_statistics[params][param])))
                print ("MSEy :  {}").format(flag_MSEy_sum/float(len(self.param_statistics[params][param])))
                print ("MSEs :  {}").format(flag_MSEs_sum/float(len(self.param_statistics[params][param])))

                flag_result_list = []
                flag_result_list.append(flag_avg_sum  /float(len(self.param_statistics[params][param])))
                flag_result_list.append(flag_MSEx_sum /float(len(self.param_statistics[params][param])))
                flag_result_list.append(flag_MSEy_sum /float(len(self.param_statistics[params][param])))
                flag_result_list.append(flag_MSEs_sum /float(len(self.param_statistics[params][param])))

                self.best_param.append(flag_result_list)

        best_param = 0
        best_place = 0 
        print ("\n\n")

        for results in range(0, len(self.best_param[param])):
            for param in range(0, len(self.param_statistics)):
                for data in range(0, len(self.param_statistics[param])):
                    if (data == 0):
                        best_param = self.best_param[data + data*param][results]
                        best_place = data
                    elif ( self.best_param[data + data*param][results] < best_param):
                        best_param = self.best_param[data + data*param][results]
                        best_place = data

                print ("==================================================")
                print ("=========    Best {} of type {}    =========").format(self.header[len(self.header)-self.output+results], self.header[param])
                print ("=========    param : {} , value {} ").format(self.param_size[param][best_place], best_param)
            print ("\n\n")



    def drawGraph(self):
        # make folder to save graph
        newpath = self.path + "/result"
        if not os.path.exists(newpath):
            os.makedirs(newpath)
        newpath = self.path + "/result/books_read.png"

        # print (self.output)

        for output in range(0, self.output):
            for param in range(0, len(self.param_statistics)):
                for data in range(0, len(self.param_statistics[param])):
                    for i in range (1, len(self.param_statistics[param][data])):
                        plt.plot(i, float(self.param_list[self.param_statistics[param][data][i]][len(self.header)-4+output]), marker="o", color="red")

                    self.savePlt(plt, self.header[param], float(self.param_statistics[param][data][0]), self.header[len(self.header)-4+output])

    def savePlt(self, plt, Param_, Type_, Data_):
        newpath = self.path + "/result/{}_{}_{}.png".format(Param_, Type_, Data_)
        plt.xlabel('Data Amount')
        plt.ylabel('{}'.format(Data_))
        plt.savefig(newpath)
        plt.clf()


if __name__=='__main__':
    # orb = Result('/home/ron/work/src/all_process/data/ORB')
    orb = Result('/home/ron/work/src/all_process/data/PLICP')
    print ("\n\n\n")
    print ("====================================================")
    print ("===============    Best Result    ==================")
    print ("====================================================")
    print ("\n===============     best_avg      ==================\n")
    print (orb.best_avg)
    print ("\n==============     best_MSE_x      =================\n")
    print (orb.best_MSE_x)
    print ("\n==============     best_MSE_y      =================\n")
    print (orb.best_MSE_y)
    print ("\n==============     best_MSE_s      =================\n")
    print (orb.best_MSE_s)

    # plicp = result('/home/ron/work/src/all_process/data/OLD_2/PLICP')
