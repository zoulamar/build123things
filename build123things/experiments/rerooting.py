# This source code tries to invoke `Thing. walk` to change kinematics root.

from build123things.examples.serial_manip import EndEffector, LinkBase

#print("Try walk on ee")
#for a in EndEffector().walk():
#    print(a)

print("Try walk on link base")
for a in LinkBase().walk():
    print(a)



