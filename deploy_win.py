import shutil
import sys
import os

target_dir = 'c:\\Users\\artik\\OLS\\'
target = 'c:\\Users\\artik\\OLS\\bin\\'
os.system(r'pyinstaller -i scripts\icon.ico -F scripts\ols_gui')
shutil.copyfile(r'dist\ols_gui.exe',target_dir + 'ols_gui.exe')
for f in ['astap_cli.exe',
          'ASICamera2.dll', 'SVBCameraSDK.dll',  'cppcms.dll', 'libcurl.dll',
          'opencv_core4100.dll', 'opencv_imgproc4100.dll', 'tiff.dll', 'turbojpeg.dll',
          'booster.dll', 'jpeg62.dll', 'libpng16.dll',
          'meadecam.dll',
          'opencv_imgcodecs4100.dll',
          'toupcam.dll', 'zlib.dll' ]:
    shutil.copyfile(r'c:\Users\artik\Packages\all-release\bin' + '\\' + f,target + f)

for f in ['LICENSE','copyright.txt', 'included_in_windows_distribution.txt']:
    shutil.copyfile(f,target_dir + f)
