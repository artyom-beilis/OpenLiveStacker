S=25;
N=1000;
stars=10

ar = rand(1,stars)*3.14159*2;
pos = rand(1,stars)*500;
scale = 0.5 + rand(1,stars);

fd=fopen('log.txt','w');
for n=[1:50]
    frame=zeros(N*2,N*2,3);
    for a=1:10
        x0 = cos(ar(a)) * pos(a) + N/2 ;
        y0 = sin(ar(a)) * pos(a) + N/2;
        
        r0 = N/2 + x0 + n*1.23235;
        c0 = N/2 + y0 + n*2.34253;
        dS = ceil(S / scale(a));
        for r=floor(r0) - dS:ceil(r0) + dS
            for c=floor(c0) - dS:ceil(c0) + dS
                dr = r - r0;
                dc = c - c0;
                rad = sqrt(dr^2 + dc^2) * 0.2 * scale(a);
                if rad == 0
                    v=1;
                else
                    v= (sin(rad)/rad)^2;
                end
                for color=1:3
                    frame(r,c,color) += v * (color/3);
                end
            end
        end
    end
    frame += randn(N*2,N*2,3) * 0.01;
    frame = max(0,min(1,frame));
    imwrite(imresize(frame,0.3333/2),sprintf('frame_%08d.tiff',n))
    fprintf(fd,'%d,%d\n',n,n);
end
fclose(fd);
        
