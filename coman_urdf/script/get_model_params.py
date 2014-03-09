'''
Simple script to get model info com a .xacro file. These infos can be used to automatically generate a manifest.xml for the model
'''

from bs4 import BeautifulSoup
import sys
import string
import os

if __name__ == '__main__':
  if len(sys.argv) < 3:
    print 'Error: correct usage is get_model_params file_name property_name'
  else:
    coman_config_file = open(sys.argv[1])
    soup_coman_config = BeautifulSoup(coman_config_file,"xml")

    try:
      config_el = soup_coman_config.find(attrs={"name": sys.argv[2]})
      if config_el and config_el.has_attr('value'):
        print config_el['value']

    finally:
      print('')
