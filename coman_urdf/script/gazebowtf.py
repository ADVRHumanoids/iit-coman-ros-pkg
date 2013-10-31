'''
Simple script to work around the bugs in gzsdf. Notice that there are no error checks, 
and that the script expects a newline between the <wtf> tag and its children (see coman.gazebo.wtf for an example)
'''

from bs4 import BeautifulSoup
import sys
import string


if __name__ == '__main__':
    wtf_file = open(sys.argv[1])
    soup_wtf = BeautifulSoup(wtf_file,"xml")
    sdf_file = open(soup_wtf.robot['filename'])
    soup_sdf = BeautifulSoup(sdf_file,"xml")
    for wtf_el in soup_wtf.find_all('wtf'):
        soup_el = soup_sdf.find(attrs={"name": wtf_el['reference']})
        if soup_el:
            soup_el.append(wtf_el.contents[1])
    print(soup_sdf.prettify(formatter="minimal"))

