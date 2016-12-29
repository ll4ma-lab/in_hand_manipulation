from subprocess import Popen


if __name__=='__main__':
    bag=Popen(["/bin/bash","bag_record.sh","exp_1"])
    print "hello"
    raw_input("kill")
    bag.terminate()
