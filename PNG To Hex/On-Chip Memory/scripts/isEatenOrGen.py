
func = "or"
orOp = " || "
initOp = ", "
op = orOp;
if(func == "init"):
    outFile = open("../script_out/"+ 'isEatenInitGen.txt', 'w')
elif (func == "or"):
    outFile = open("../script_out/"+ 'isEatenOrGen.txt', 'w')


dotCount = 240

def genFunc(func, op):
    string = ""
    initString = "logic isEaten,"
    orString = "isEaten = "
    if(func == "init"):
        string = initString
    elif (func == "or"):
        string = orString
    for i in range(dotCount):
        if (i == dotCount-1):
            string += "isEaten" + str(i) +";"
        else:
            string += "isEaten"+ str(i) + op
    return string



res = genFunc(func, op)
outFile.write(res)
outFile.close()
