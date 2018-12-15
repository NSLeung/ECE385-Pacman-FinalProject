# outFile = open("../script_out/"+ 'if_isEaten_gen.txt', 'w')
# outFile = open("../script_out/"+ 'isEaten_param_gen.txt', 'w')
outFile = open("../script_out/"+ 'isEaten_module_inst_gen.txt', 'w')

string = ""
dotCount = 240
for i in range(1,dotCount+1):
    # if (i == dotCount):
    # string += "if(isEaten" + str(i) + "==1'b1)\n"+"begin\n" + "\tmem_address = 16'd000;\n" + "end\n"
    # else:
    #     string += "isEaten"+ str(i) + op
    string+=".isEaten"+str(i)+"(isEaten"+str(i)+"),\n"
# print(string)
outFile.write(string)
outFile.close()
