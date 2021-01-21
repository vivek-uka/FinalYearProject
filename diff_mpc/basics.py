import autograd.numpy as np

def delete_append(u):
    u = np.delete(u, 0, axis=1)
    u = np.append(u, [[0], [0]], axis=1)
    return u


def dist(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def minimum(data):

    mini = float('inf')

    for i in range(len(data)):
        if data[i] < mini:
            mini = data[i]
            index = i

    return index

def perpdist2D(a,b,c,x,y):
    return abs(a*x + b*y + c)/np.sqrt(a**2 + b**2)

def perpdist3D(x1, y1, z1, x2, y2, z2, x, y, z):
    a = x1 - x2
    b = y1 - y2
    c = z1 - z2

    d = a*x + b*y + c*z

    denom = a*(x1 - x2) + b*(y1 - y2) + c*(z1 - z2)
    num = d - a*x1 - b*y1 - c*z1
    t = (num)/(denom)
    xx = (x1 - x2) * t + x1
    yy = (y1 - y2) * t + y1
    zz = (z1 - z2) * t + z1

    return np.sqrt((xx - x)**2 + (yy - y)**2 + (zz - z)**2)