import sys


if len(sys.argv) > 1:
    if sys.argv[1] == 'new':
        print 'true'
    else:
        print 'false'
else:
    print 'false'
