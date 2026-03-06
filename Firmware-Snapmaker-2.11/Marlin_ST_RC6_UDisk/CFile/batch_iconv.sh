#!/bin/bash
set +x

 for file in `find . -name "*.cpp" -o -name "*.c"  -o -name "*.h"`
do 
    echo $file
    #if [ `$file` -ne 'Marline/language_test.h' ]; then
        if [ `file -I $file | grep utf-8 | wc -l` -eq '0' ]; then
            iconv -f GBK -t UTF-8 <$file >$file.bak
            mv $file.bak $file
        fi
    #fi

 done
