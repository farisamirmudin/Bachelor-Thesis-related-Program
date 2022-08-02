import numpy as np
from sympy import *
from sympy.geometry import *

def trilateration(points, distances, dim):
    with open('parameters/abc_param.txt', 'r') as f:
        lines = f.readlines()
        a=float(lines[0][4:])
        b=float(lines[1][4:])
        c=float(lines[2][4:])
    if dim == 3:
        p1,p2,p3,p4 = np.array(points)
        r1,r2,r3,r4 = np.array(distances)
    else:
        p1,p2,p3 = np.array(points)
        r1,r2,r3 = np.array(distances)
    e_x=(p2-p1)/np.linalg.norm(p2-p1)
    i=np.dot(e_x,(p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    e_z=np.cross(e_x,e_y)
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    if dim == 2:
        ans1=p1+(x*e_x)+(y*e_y)
        return ans1
    z1=np.sqrt(max(0, r1**2-x**2-y**2))
    z2=-z1
    ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
    ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)
    multp_z = lambda x:a*x**2+b*x+c
    ans1[2] = multp_z(ans1[2])
    return ans1
    # dist1=np.linalg.norm(p4-ans1)
    # dist2=np.linalg.norm(p4-ans2)
    # if np.abs(r4-dist1)<np.abs(r4-dist2):
    #     return ans1
    # else:
    #     return ans2

    
    


