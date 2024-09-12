from math import *


D = 1

def XZ(i, theta, phi):
    Di = D/2 if i % 2 == 1 else D
    return (Di * cos(theta + phi), Di * sin(theta+phi))


if __name__ == '__main__':
    phi = pi/6.0
    for fid in range(0,3):
        f = open('./yarn_'+str(fid)+'.txt','w')
        for i in range(6):
            theta = pi/3.0 * i
            x,z = XZ(i,theta,phi)
            f.write(f'{x},{i},{z}\n')
        phi +=  pi * 2.0/3.0
        f.close()
    
    phi = pi/6.0
    for fid in range(3,6):
        f = open('./yarn_'+str(fid)+'.txt','w')
        for i in range(6):
            theta = -pi/3.0 * i
            x,z = XZ(i+1,theta,phi)
            f.write(f'{x},{i},{z}\n')
        phi += pi * 2.0/3.0
        f.close()