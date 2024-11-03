import csv
import sys
import re
import json
zeros=re.compile('^(.*)(([A-Z]+)[ 0]+)([^0].*)$')

def parse_ra(val):
    h,m,s = val.split(':')
    return 15*(int(h)  + (60*int(m) + float(s))/3600)

def parse_de(val):
    d,m,s = val.split(':')
    deg = int(d)
    sign = 1 if deg >= 0 else -1
    return sign*(abs(deg)  + (60*int(m) + float(s))/3600)

def norm_rade(v):
    return v.split('.')[0]

def normalize_name(name):
    m=zeros.match(name)
    if m:
       new_name = m.group(1) + m.group(3) + m.group(4)
       return normalize_name(new_name)
    return name.strip().upper()


def get_OpenNGC_DSO():
    # M45 is missing
    result={'M45':['03:47:24','+24:07:00']}
    messier_row = 23
    ic_row = 25
    with open('external/OpenNGC/NGC.csv','r') as f:
        mapped = set(range(1,111));
        for i,row in enumerate(csv.reader(f,delimiter=';')):
            if i<=1:
                continue
            object_id = row[0]
            ra = norm_rade(row[2])
            de = norm_rade(row[3])
            size = 0 if row[5]=='' else float(row[5])
            messier = int(row[messier_row]) if row[messier_row]!='' else 0
            ics=[]
            if row[ic_row]!='':
                ics = row[ic_row].split(',')
            #hack data
            if object_id == 'NGC5866':
                messier = 102
            result[normalize_name(object_id)] = [ra,de]
            if messier:
                result['M%d' % messier] = [ra,de]
            for ic in ics:
                result[normalize_name('IC%s' % ic)] = [ra,de]
    return result

def split_RA(RA):
    rh = int(RA);
    rs = int(RA*3600) % 3600;
    rm = rs / 60;
    rs = rs % 60;
    return (rh,rm,rs)

def split_DEC(DEC):
    s = 1;
    if DEC < 0:
        s=-1;
        DEC=-DEC;
    dd = int(DEC);
    ds = int(DEC*3600) % 3600;
    dm = ds / 60;
    ds = ds % 60;
    dd = dd * s;
    return (dd,dm,ds)

def format_RA(RA):
    return "%02d:%02d:%02d" % split_RA(RA)

def format_DEC(DEC):
    return "%02d:%02d:%02d" % split_DEC(DEC)

def get_stars(allstars):
    with open('external/western_constellations_atlas_of_space/data/hygdata_v3/hygdata_v3.csv','r') as f:
        for i,row in enumerate(csv.reader(f)):
            if i <= 1:
                continue
            sid = row[1]
            name=row[6]
            if name == '':
                continue
            ra=format_RA(float(row[7]))
            de=format_DEC(float(row[8]))
            mag=float(row[13])
            if mag <= 5:
                allstars[normalize_name(name)] = [ra,de]

def make_vsop87(out_path):
    with open(out_path,"wb") as f:
        with open('external/vsop87-multilang/Languages/JavaScript/vsop87a_large.js','rb') as f1:
            f.write(f1.read())
        with open('external/vsop87-multilang/Languages/JavaScript/vsop87a_large_velocities.js','rb') as f2:
            f.write(f2.read())
        f.write(b'vsop87a_full = vsop87a_large;\n')
        f.write(b'vsop87a_full_velocities = vsop87a_large_velocities;\n')
        with open('external/vsop87-multilang/Languages/JavaScript/CelestialProgrammingReduce.js','rb') as f3:
            f.write(f3.read())


if __name__ == "__main__":
    make_vsop87(sys.argv[2])
    db = get_OpenNGC_DSO()
    get_stars(db)
    with open(sys.argv[1],'w') as f:
        f.write('// Attribution-ShareAlike 4.0 International\n')
        f.write('// generated from https://github.com/mattiaverga/OpenNGC\n')
        f.write('var jsdb= {\n');
        keys = list(db)
        for i,k in enumerate(keys):
            if i == len(keys)-1:
                sep = '\n'
            else:
                sep = ',\n'
            f.write('"%s":%s%s' % (k,json.dumps(db[k]),sep))
        f.write('};\n')
