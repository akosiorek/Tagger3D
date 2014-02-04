'''
Created on 22 sie 2013

@author: adam
'''

import sys, getopt, os, exceptions, random

def main():
    
    try:
        
        opts, args = getopt.getopt(sys.argv[1:], "r:p:")
    except getopt.GetoptError:
        
        print "python imgListPrep.py -r <rootDir> -p <trainset percentage in the dataset>"
        sys.exit(2)
        
    if len(opts) < 2:
        print "python imgListPrep.py -r <rootDir> -p <trainset percentage in the dataset>"
        sys.exit(2)
        
    for opt, arg in opts:
        if opt == "-r":
            root = arg
        if opt == "-p":
            ratio = float(arg)
         
    catFolders = os.listdir(root)
    imgsByCat = []
    os.chdir(root)
    for folder in catFolders:
        
        listOfDirs = os.listdir(folder)
        random.shuffle(listOfDirs)
        newList = []
        for dir in listOfDirs:
            newList.append(os.path.join(root, folder, dir))          
        imgsByCat.append(newList)
        
    trainset = open("train.txt", "w")
    testset = open("test.txt", "w")
    trainlabels = open("train_labels.txt", "w")
    testlabels = open("test_labels.txt", "w")
    
    cat = 0
    for imgs in imgsByCat:
        
        trainsetSize = int(len(imgs) * ratio) 
        trainImgs = imgs[0:trainsetSize]
        testImgs = imgs[trainsetSize + 1 : len(imgs) - 1]
        for img in trainImgs:
            trainset.write(img + "\n")
            trainlabels.write(str(cat) + "\n")
            
        for img in testImgs:
            testset.write(img + "\n")
            testlabels.write(str(cat) + "\n")
        cat += 1
        
    trainset.close()
    testset.close()
    trainlabels.close()
    testlabels.close()
        
if __name__ == "__main__":
    main()
    
    