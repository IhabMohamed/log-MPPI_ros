#!/usr/bin/env python
import sys, os

'''This module is used to fork the current process into a daemon.

Almost none of this is necessary (or advisable) if your daemon is being started
by inetd. In that case, stdin, stdout and stderr are all set up for you to
refer to the network connection, and the fork()s and session manipulation
should not be done (to avoid confusing inetd). Only the chdir() and umask()
steps remain useful.

References:

    UNIX Programming FAQ
        1.7 How do I get my program to act like a daemon?
        http://www.unixguide.net/unix/programming/1.7.shtml
        http://www.faqs.org/faqs/unix-faq/programmer/faq/

    Advanced Programming in the Unix Environment
        W. Richard Stevens, 1992, Addison-Wesley, ISBN 0-201-56317-7.

$Id: daemonize.py 57 2008-12-22 16:05:53Z wroniasty $
'''

def daemonize (stdin='/dev/null', stdout='/dev/null', stderr='/dev/null'):

    '''This forks the current process into a daemon. The stdin, stdout, and
    stderr arguments are file names that will be opened and be used to replace
    the standard file descriptors in sys.stdin, sys.stdout, and sys.stderr.
    These arguments are optional and default to /dev/null. Note that stderr is
    opened unbuffered, so if it shares a file with stdout then interleaved
    output may not appear in the order that you expect. '''

    # Do first fork.
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)   # Exit first parent.
    except OSError, e:
        sys.stderr.write ("fork #1 failed: (%d) %s\n" % (e.errno, e.strerror) )
        sys.exit(1)

    # Decouple from parent environment.
    os.chdir("/")
    os.umask(0)
    os.setsid()

    # Do second fork.
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)   # Exit second parent.
    except OSError, e:
        sys.stderr.write ("fork #2 failed: (%d) %s\n" % (e.errno, e.strerror) )
        sys.exit(1)

    # Now I am a daemon!

    # Redirect standard file descriptors.
    si = open(stdin, 'r')
    so = open(stdout, 'w')
    se = open(stderr, 'w', 0)
    os.dup2(si.fileno(), sys.stdin.fileno())
    os.dup2(so.fileno(), sys.stdout.fileno())
    os.dup2(se.fileno(), sys.stderr.fileno())

