# This source code tries to invoke `Thing. walk` to change kinematics root.

from build123things.examples.serial_manip import EndEffector, LinkBase

#print("Try walk on ee")
#for a in EndEffector().walk():
#    print(a)

print("Try in reverse")
obj_root = LinkBase()
pen = obj_root.shoulder.humerus.forearm.ee.cap.pen
for a in pen.walk():
    ...


