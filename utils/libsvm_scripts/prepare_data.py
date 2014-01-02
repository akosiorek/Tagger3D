#!/usr/bin/python

folder = "tokyo"

test = "/home/adam/workspace/Tagger3D/models/" + folder + "/easy/testHistogram"
train = "/home/adam/workspace/Tagger3D/models/" + folder + "/easy/trainHistogram"
ext = ".easy"

def scaleData(inpath, outpath):
    lines = [line.rstrip().split(" ") for line in open(inpath)]
    for line in lines:
        for i in xrange(len(line) - 1, 0, -1):
            if line[i] == '0':
                del line[i]
            else:
                line[i] = str(i) + ":" + line[i]
                
    with open(outpath, 'w') as f:
        for line in lines:
            for el in line:
                f.write(el + " ")
            f.write("\n")

if __name__ == "__main__":
    scaleData(train, train + ext)
    scaleData(test, test + ext)
   
