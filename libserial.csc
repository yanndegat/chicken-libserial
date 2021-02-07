(module libserial (detect-serial)
        (import scheme)
        (import chicken.base)
        (import chicken.file)
        (import chicken.fixnum)
        (import chicken.foreign)
        (import chicken.string)

        (import srfi-69) ;; hash table
        (import srfi-13) ;; strings

        (foreign-declare #<<EOF
#define SERIALTIMEOUT 10

#include <stdio.h>
#include <string.h>

#include <sys/file.h>

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>


// baud rates
#define RS232_110BAUD     110
#define RS232_300BAUD     300
#define RS232_600BAUD     600
#define RS232_1200BAUD    1200
#define RS232_2400BAUD    2400
#define RS232_4800BAUD    4800
#define RS232_9600BAUD    9600
#define RS232_19200BAUD   19200
#define RS232_38400BAUD   38400
#define RS232_14400BAUD   14400
#define RS232_28800BAUD   28800
#define RS232_57600BAUD   57600
#define RS232_76800BAUD   76800
#define RS232_115200BAUD  115200

// parity
#define RS232_NOPARITY 0
#define RS232_ODDPARITY 1
#define RS232_EVENPARITY 2

// stop bits
#define RS232_ONESTOPBIT 1
#define RS232_TWOSTOPBITS 2

// data bits
#define RS232_5BITS 5
#define RS232_6BITS 6
#define RS232_7BITS 7
#define RS232_8BITS 8

// function prototypes
int serial_open(char *dev, int baudrate, int bitsize, int parity, int stopbits);
int serial_openfile(char *filepath);
void serial_close(int dev);
void serial_writechar(int dev, int val);
int serial_readchar(int dev);
int serial_error();
int serial_timeout();
void serial_flush(int dev);
int serial_getDTR(int dev);
void serial_setDTR(int dev, int val);
int serial_getRTS(int dev);
void void_setRTS(int dev, int val);

// error codes
static int _serial_error, _serial_notready;

// open the serial device
int serial_open(char *dev, int baudrate, int bitsize, int parity, int stopbits){
  _serial_error=_serial_notready=0;

  struct termios my_termios;
  int fd=0,spd;

  fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK );
  // this blocks if already locked by another process:
  // flock(fd,LOCK_EX);
  // try to prevent blocking:
  if (flock(fd,LOCK_EX|LOCK_NB)==-1) { _serial_error=1; return 0; }
  if (fd<0) { _serial_error=1; return 0; }

  bzero(&my_termios, sizeof(my_termios));

  // configuring OPENBSD/LINUX/MACOSX serial communication:
  // misc control settings
  my_termios.c_iflag= IGNBRK;
  my_termios.c_oflag= 0;
  my_termios.c_lflag= 0;
  my_termios.c_cflag = CREAD | CLOCAL ;

  // set the baudrate, falling back to 9600
  switch (baudrate) {
  case RS232_110BAUD: spd = B110; break;
  case RS232_300BAUD: spd = B300; break;
  case RS232_600BAUD: spd = B600; break;
  case RS232_1200BAUD: spd = B1200; break;
  case RS232_2400BAUD: spd = B2400; break;
  case RS232_4800BAUD: spd = B4800; break;
  case RS232_9600BAUD: spd = B9600; break;
  case RS232_19200BAUD: spd = B19200; break;
  case RS232_38400BAUD: spd = B38400; break;
  case RS232_57600BAUD: spd = B57600; break;
  case RS232_115200BAUD: spd = B115200; break;
  default: spd = B9600; break;
  }
  cfsetospeed(&my_termios, (speed_t)spd);
  cfsetispeed(&my_termios, (speed_t)spd);

  // parity, falling back on none
  my_termios.c_cflag &= ~(PARENB | PARODD);
  switch (parity) {
  case RS232_EVENPARITY:
    my_termios.c_cflag |= PARENB;
    break;
  case RS232_ODDPARITY:
    my_termios.c_cflag |= PARODD;
    break;
  }

  // stopbits, falling back on 1
  switch (stopbits) {
  case RS232_TWOSTOPBITS:
    my_termios.c_cflag |= CSTOPB;
    break;
  }

  // enable hardware flow control
  //  my_termios.c_cflag |= CRTSCTS;

  // bits, falling back on 8
  switch (bitsize) {
  case RS232_5BITS: my_termios.c_cflag = ( my_termios.c_cflag & ~CSIZE) | CS5; break;
  case RS232_6BITS: my_termios.c_cflag = ( my_termios.c_cflag & ~CSIZE) | CS6; break;
  case RS232_7BITS: my_termios.c_cflag = ( my_termios.c_cflag & ~CSIZE) | CS7; break;
  default: my_termios.c_cflag = ( my_termios.c_cflag & ~CSIZE) | CS8; break;
  }

  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &my_termios)) {
    //printf("ERROR: serial: tcsetattr() failed\n");
  }

  return (int)fd;
}

// open a connection to a file
int serial_openfile(char *filepath){
  return open(filepath, O_RDWR);
}

void serial_close(int d){
  _serial_error=_serial_notready=0;
  int fd=d;
  if (!fd) { _serial_error=1;  return; }
  if (close(fd)) {
    //printf("ERROR: serial: close failed err=%i [%s]\n",errno,strerror(errno));
    _serial_error=1;
  }
}

// write data to the serial port
void serial_writechar(int d, int val){
  unsigned char buf = (unsigned char)val;
  _serial_error=_serial_notready=0;

  int fd=d;
  if (!fd) { _serial_error=1; return; }
  ssize_t n_written=0;
  n_written = write(fd,&buf,1);
  if (n_written!=1) {
    // printf("writechar: errno=%i [%s]\n",errno,strerror(errno));
    _serial_error=1;
  }
}

// read data from the serial port
int serial_readchar(int d){
  unsigned char buf[1]={0};
  _serial_error=_serial_notready=0;

  int fd=d;
  if (read(fd,&buf,1)!=1) {
    if (errno==35) {
      _serial_notready=1;
    }else {
      _serial_error=1;
      // printf("readchar: errno=%i [%s]\n",errno,strerror(errno));
    }
  }
  return (int)buf[0];
}

void serial_flush(int d){
  int fd=d;
  tcflush(fd, TCIOFLUSH);
}

// Change and query control lines
int serial_getDTR(int d){
  _serial_error=_serial_notready=0;
  int fd=d;
  if (!fd) { _serial_error=1;  return 0; }
  int serial;
  ioctl(fd, TIOCMGET, &serial);
  return (serial & TIOCM_DTR);
}

int serial_getRTS(int d){
  _serial_error=_serial_notready=0;
  int fd=d;
  if (!fd) { _serial_error=1;  return 0; }
  int serial;
  ioctl(fd, TIOCMGET, &serial);
  return (serial & TIOCM_RTS);
}

void serial_setDTR(int d, int s){
  _serial_error=_serial_notready=0;
  int fd=d;
  if (!fd) { _serial_error=1;  return; }
  if (s==0) {
    ioctl(fd, TIOCMBIC, TIOCM_DTR);
  } else {
    ioctl(fd, TIOCMBIS, TIOCM_DTR);
  }
}

void serial_setRTS(int d, int s){
  _serial_error=_serial_notready=0;
  int fd=d;
  if (!fd) { _serial_error=1;  return; }
  if (s==0) {
    ioctl(fd, TIOCMBIC, TIOCM_RTS);
  } else {
    ioctl(fd, TIOCMBIS, TIOCM_RTS);
  }
}

int serial_error(void) { return _serial_error; }
int serial_timeout(void) { return _serial_notready; }
EOF
                         )

(define serial:open (foreign-lambda int "serial_open" c-string int int int int))
(define serial:openfile (foreign-lambda int "serial_openfile" c-string))
(define serial-close (foreign-lambda void "serial_close" int))

(define serial-readchar (foreign-lambda int "serial_readchar" int))
(define serial-writechar (foreign-lambda void "serial_writechar" int int))
(define serial-flush (foreign-lambda void "serial_flush" int))

(define serial:error (foreign-lambda int "serial_error"))
(define serial:timeout (foreign-lambda int "serial_timeout"))
(define (serial-error) (not (fx= (serial:error) 0)))
(define (serial-timeout) (not (fx= (serial:timeout) 0)))

(define serial-dtr-set! (foreign-lambda void "serial_setDTR" int bool))
(define serial-dtr-get (foreign-lambda bool "serial_getDTR" int))
(define serial-rts-set! (foreign-lambda void "serial_setRTS" int bool))
(define serial-rts-get (foreign-lambda bool "serial_getRTS" int))


;; ----------------------
;; device autodetection
;; list of devices to autodect on - this is platform dependent...
(define serial:devicelist '("/dev/ttyUSB0" "/dev/ttyU0" "/dev/ttyS0"))
(define serial:ratelist   '(9600 19200 38400 115200))

;; autodect list
;;  (baudrate databits parity stopbits)
(define serial:commlist '(
  (9600   8 0 1)
  (19200  8 0 1)
  (19200  8 2 1)
  (38400  8 0 1)
  (115200 8 0 1)
))
(define (serial-try devname baudrate databits parity stopbits testproc)
  (let ((dev (serial:open devname baudrate databits parity stopbits)))
    (if (or (serial-error) (serial-timeout))
      #f
      (if testproc
        (if (testproc dev)
          dev
          (begin (serial-close dev) #f)
        )
        dev
      )
    )
  ))

(define (serial-tryfile filepath)
  (let ((dev (serial:openfile filepath)))
     (if (or (serial-error) (serial-timeout)) #f
        dev)))


(define (serial-autodetect devname testproc)
;;  (warning "serial: starting device detection..")
  (let loop ((d (if devname (list devname) serial:devicelist)))
    (if (fx= (length d) 0)
      (begin
        (warning "serial: device autodetection failed")
        #f
      )
      (let ((res (let loop2 ((r serial:commlist))
        (if (fx= (length r) 0)
          #f
          (let ((dev (serial-try (car d) (car (car r))
                (cadr (car r)) (caddr (car r)) (cadddr (car r)) testproc)))
            (if dev
              (begin
                (print (string-append "serial: found device at " (car d)
                                           " using " (number->string (car (car r)))
                                           " " (number->string (cadr (car r)))
                                           (if (fx= (caddr (car r)) 0) "N" (if (fx= (caddr (car r)) 1) "O" "E"))
                                           (number->string  (cadddr (car r)))
                ))
                dev
              )
              (loop2 (cdr r))
            )
          )
        ))))
        (if res res (loop (cdr d)))
      )
    )
  ))


;; Find a usb-serial converter on macos and linux
(define (detect-serial)
  (let ((serial-file=? (lambda (f) (or (string-contains f "tty")
                 (or (string-contains-ci f "serial") (string-contains-ci f "usb"))))))
       (find-files "/dev" limit: 0 test: serial-file=?)))

;; rs232 communication protocol often define a start and end character
;; this generic cache system can capture such delimited messages
(define serial:cache (make-hash-table))

(define (serial-cache-setup dev char1 char2)
  (hash-table-set! serial:cache dev (list char1 char2 #f)))

(define (serial-cache-clear dev)
  (let* ((entry (hash-table-ref/default serial:cache dev #f))
         (c1  (if entry (car entry) #f))
         (c2  (if entry (cadr entry) #f)))
    (if entry
      (hash-table-set! serial:cache dev (list c1 c2 #f))
      (warning "serial-cache-clear: cache not initialized"))
  ))

(define (serial-cache-read dev . hook)
  (let* ((entry (hash-table-ref/default serial:cache dev #f))
         (c1  (if entry (car entry) #f))
         (c2  (if entry (cadr entry) #f))
         (buf (if entry (caddr entry) #f))
         (hookproc (if (fx= (length hook) 1) (car hook) #f)))
    (if entry
      (call/cc (lambda (return)
        ;; step 1: seek start char
        (if (not buf)
          (let loop ()
            (let ((c (serial-readchar dev)))
              (if (or (serial-timeout) (serial-error)) (return #f))
              (if hookproc (hookproc c))
              (if (if (procedure? c1) (c1 c) (fx= c c1)) (set! buf (string (integer->char c))) (loop))
            )
          )
        )
        ;; step 2: seek end char
        (let loop2 ((r (if buf buf "")))
          (let ((c (serial-readchar dev)))
            (if (or (serial-timeout) (serial-error)) (begin
               (hash-table-set! serial:cache dev (list c1 c2 r))
               (return #f)
            ))
            (if hookproc (hookproc c))
            (if (if (procedure? c2) (c2 c (fx+ (string-length r) 1)) (fx= c c2)) (begin
              (serial-cache-clear dev)
              (return (if (fx<= (string-length r) 1)
                #f
                (string-append r (string (integer->char c)))
              ))
            ))
            (loop2 (string-append r (string (integer->char c))))
          )
        )
      ))
      (begin
        (warning "serial-cache-read: cache not initialized")
        #f
      )
    )
  ))
    )

(import libserial)
(import chicken.string)
(print (detect-serial))
