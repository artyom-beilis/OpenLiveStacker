
import numpy as np

def split4(all):
    R=all[::2,::2]
    G1=all[::2,1::2]
    G2=all[1::2,0::2]
    B=all[1::2,1::2]
    return R,G1,G2,B

def print_mat(v,pad=0):
    print('{')
    for r in v:
        vals = r.tolist()
        if pad:
            vals.append(0)
        for a in vals:
            print('%5d,' % a, end=' ')
        print()
    print('}')
    

def meanv(v):
    v1,v2,v3,v4 = split4(v);
    return (2+v1+v2+v3+v4) / 4

all = (np.random.rand(8,12) * 10000).astype(np.uint32)

R,G1,G2,B = split4(all)
R=meanv(R)
G1=meanv(G1)
G2=meanv(G2)
B=meanv(B)

res=np.zeros((4,6),dtype=np.uint32)
res[::2,::2] = R
res[::2,1::2] = G1
res[1::2,0::2] = G2
res[1::2,1::2] = B


print_mat(all,pad=1)
print_mat(res)

