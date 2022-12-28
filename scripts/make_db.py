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
    return name


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

if __name__ == "__main__":
    db = get_OpenNGC_DSO()
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
