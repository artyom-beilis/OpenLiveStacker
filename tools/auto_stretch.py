from imageio import imread,imsave
import numpy as np
import sys
import math
from math import log


for path in sys.argv[1:]:
    bins = 1024
    mean_target = 0.25
    top_per = 99.99
    first_above = 0.1
    gamma_limit = 4
    linear_point = 0.008;

    img = imread(path).astype(np.float32)
    img = img * (1.0 / np.max(img))
    h,w,c = img.shape
    r0 = h // 8
    r1 = h - h // 8
    c0 = w // 8
    c1 = w - w // 8
    wb = np.mean(img[r0:r1,c0:c1,:],axis=(0,1),keepdims=True)
    wb = np.max(wb) / wb
    print("WB:",wb.reshape(-1))
    img = np.minimum(1.0,img*wb)
    top_point = np.percentile(img[r0:r1,c0:c1,:],top_per)
    hist = np.histogram(img[r0:r1,c0:c1,:],bins=bins,range=(0.0,1.0))[0]
    hist = (1.0/np.sum(hist)) * hist.astype(np.float32)
    dhist = hist[1:] - hist[:-1] 
    maxv = np.max(dhist[0:bins//2])
    low_point = np.where(dhist >= first_above *maxv)[0][0] / bins
    img = np.maximum(0,np.minimum(1,(img-low_point) / (top_point- low_point)))
    meanv = np.mean(img[r0:r1,c0:c1,:])
    gamma_correction = max(1.0,min(gamma_limit,log(meanv)/log(mean_target)))

    print(gamma_correction)
    half_point = 0.5**(gamma_correction)
    print("HP=",half_point)
    print(1.0/(2.0*half_point))
    print(math.acosh(1.0/(2.0*half_point)))
    print(math.sinh(math.acosh(1.0/(2.0*half_point))))
    a = 1/half_point * math.sinh(math.acosh(1.0/(2.0*half_point)))
    print('Stretch [%f->%f]^1/%f' % (low_point,top_point,gamma_correction))
    
    linear_gain = (linear_point**(1/gamma_correction)) / linear_point;
    img_gamma = np.minimum(img**(1/gamma_correction),img*linear_gain)
    linear_gain = math.asinh(a*linear_point)/math.asinh(a) / linear_point
    img_asinh = np.minimum(np.arcsinh(a*img)*(1/math.asinh(a)),img*linear_gain)
    img_gamma = (img_gamma*255).astype(np.uint8)
    img_asinh = (img_asinh*255).astype(np.uint8)
    imsave(path.replace('.tiff','_stretched_gamma.png'),img_gamma)
    imsave(path.replace('.tiff','_stretched_asinh.png'),img_asinh)



