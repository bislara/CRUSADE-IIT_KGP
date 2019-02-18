import numpy as np
import cv2 as cv
import serial
import math
import struct
import time

def nothing(val):
    pass

roi_stat=0
def roi(orig,title,defminheight,defmaxheight,defminwidth,defmaxwidth):
    global roi_stat
    height,width,channels=orig.shape
    if roi_stat<=4:
        cv.namedWindow(title)
        cv.createTrackbar('Min height',title,defminheight,height,nothing)
        cv.createTrackbar('Max height',title,defmaxheight,height,nothing)
        cv.createTrackbar('Min width',title,defminwidth,width,nothing)
        cv.createTrackbar('Max width',title,defmaxwidth,width,nothing)
        roi_stat+=1
    minheight=cv.getTrackbarPos('Min height',title)
    maxheight=cv.getTrackbarPos('Max height',title)
    minwidth=cv.getTrackbarPos('Min width',title)
    maxwidth=cv.getTrackbarPos('Max width',title)
    orig=orig[minheight:maxheight,minwidth:maxwidth]
    cv.imshow(title,orig)
    return orig

#led detection starts
ledin=False
ctr=0
ledmode='F'
led_detection_stat=0

def led_detection(orig):
    global ledin,ctr,ledmode,led_detection_stat,dir
    height,width,channels=orig.shape
    if led_detection_stat==0:
        cv.namedWindow('LED Detection')
        cv.createTrackbar('Low H', 'LED Detection' , 84, 179, nothing)
        cv.createTrackbar('High H', 'LED Detection' , 127, 179, nothing)
        cv.createTrackbar('Low S', 'LED Detection' , 21, 255, nothing)
        cv.createTrackbar('High S', 'LED Detection' , 188, 255, nothing)
        cv.createTrackbar('Low V', 'LED Detection' , 219, 255, nothing)
        cv.createTrackbar('High V', 'LED Detection' , 255, 255, nothing)
        led_detection_stat=1
    low_H=cv.getTrackbarPos('Low H', 'LED Detection')
    low_S=cv.getTrackbarPos('Low S', 'LED Detection')
    low_V=cv.getTrackbarPos('Low V', 'LED Detection')
    high_H=cv.getTrackbarPos('High H', 'LED Detection')
    high_S=cv.getTrackbarPos('High S', 'LED Detection')
    high_V=cv.getTrackbarPos('High V', 'LED Detection')
    orig=roi(orig,'LED detection ROI Selector',0,136,0,width) #357,480,5,205
    orig=cv.GaussianBlur(orig,(5,5),0)
    hsv = cv.cvtColor(orig, cv.COLOR_BGR2HSV)
    ledthresh= cv.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))
    ledthresh=cv.morphologyEx(ledthresh,cv.MORPH_OPEN,(5,5))
    ledthresh=cv.morphologyEx(ledthresh,cv.MORPH_CLOSE,(5,5))
    nonzeropix=	cv.countNonZero(ledthresh)
    cv.putText(ledthresh, str(nonzeropix), (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)

    if nonzeropix>50 and ledin==False:
        print('LED enters the frame')
        cv.putText(ledthresh, 'LED enters the frame', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
    	ledin=True

    elif nonzeropix<5 and ledin==True:
        ctr+=1
        print('LED exits the frame: ',ctr)
        cv.putText(ledthresh, 'LED exits the frame', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
        if ctr>=40:
            ledin=False
            ctr=0
            print('LED detected')
            ledmode='T'
            set_data()
            print(dir)
            serial_com()

    else:
        ctr=0
    cv.imshow('LED Detection',ledthresh)
#led detection ends

#blink_freq starts
startt=lfreq=rfreq=onr=onl=printed=blink_freq_stat=freq_count=0
freq_being_found=False
centroids=[]
timer_started=False


def blink_freq(orig):
    #global ledin,ctr,ledmode,low_H,low_S,low_V,high_H,high_S,high_V
    global angle,startt,lfreq,rfreq,onr,onl,printed,freq_being_found,centroids,timer_started,blink_freq_stat, pred_turn
    height,width,channels=orig.shape
    if blink_freq_stat==0:
        cv.namedWindow('LED Blinking Frequency')
        cv.namedWindow('Left half')
        cv.namedWindow('Right half')
        cv.createTrackbar('Low H', 'LED Blinking Frequency' , 11, 179, nothing)
        cv.createTrackbar('High H', 'LED Blinking Frequency' , 35, 179, nothing)
        cv.createTrackbar('Low S', 'LED Blinking Frequency' , 0, 255, nothing)
        cv.createTrackbar('High S', 'LED Blinking Frequency' , 14, 255, nothing)
        cv.createTrackbar('Low V', 'LED Blinking Frequency' , 229, 255, nothing)
        cv.createTrackbar('High V', 'LED Blinking Frequency' , 255, 255, nothing)
        blink_freq_stat=1
    low_H=cv.getTrackbarPos('Low H', 'LED Blinking Frequency')
    low_S=cv.getTrackbarPos('Low S', 'LED Blinking Frequency')
    low_V=cv.getTrackbarPos('Low V', 'LED Blinking Frequency')
    high_H=cv.getTrackbarPos('High H', 'LED Blinking Frequency')
    high_S=cv.getTrackbarPos('High S', 'LED Blinking Frequency')
    high_V=cv.getTrackbarPos('High V', 'LED Blinking Frequency')
    height,width,channels=orig.shape
    orig=roi(orig,'LED blinking ROI Selector',368,height,0,width) #357,480,5,205
    height,width,channels=orig.shape
    orig=cv.GaussianBlur(orig,(5,5),0)
    hsv = cv.cvtColor(orig, cv.COLOR_BGR2HSV)
    ledthresh= cv.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))
    ledthresh=cv.morphologyEx(ledthresh,cv.MORPH_OPEN,(5,5))
    ledthresh=cv.morphologyEx(ledthresh,cv.MORPH_CLOSE,(5,5))

    if freq_being_found==False:
        M=cv.moments(ledthresh)
        if M['m00']!=0:
            cX = int(M['m10']/M['m00'])
            cY = int(M["m01"] / M["m00"])
            cv.circle(ledthresh, (cX, cY), 3, (255, 255, 255), -1)
            print('cY/height: ',float(cY)/height)
            if cY>0.6*height: #and timer_started==False:
                #startt=time.time()
                freq_being_found=True
                freq_count+=1
            #timer_started=True
        cv.imshow('LED Blinking Frequency',ledthresh)

    else:
        if len(centroids)<2:
            print('Finding midline')
            im2, contours, hierarchy = cv.findContours(ledthresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            if len(contours)!=0:
                for z in contours:
                    if cv.contourArea(z)>50:
                        cv.drawContours(ledthresh, [z], 0, (0,255,0), 3)
                        M=cv.moments(z)
                        if M['m00']==0:
                            continue
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv.putText(ledthresh, str(cv.contourArea(z)), (cx,cy), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)

                        if len(centroids)==0:
                            centroids.append((cx,cy))
                        elif abs(centroids[0][0]-cx)>20 and len(centroids)==1:
                            centroids.append((cx,cy))
                            print('Midline found')
                            break

        else:
            midline=int((centroids[0][0]+centroids[1][0])/2)
            lefthalforig=orig[0:height,0:midline]
            righthalforig=orig[0:height,midline:width]
            lefthalf=ledthresh[0:height,0:midline]
            righthalf=ledthresh[0:height,midline:width]

            cv.imshow('Left half original',lefthalforig)
            cv.imshow('Right half original',righthalforig)

            leftnonzero=cv.countNonZero(lefthalf)
            rightnonzero=cv.countNonZero(righthalf)
            cv.putText(lefthalf, str(leftnonzero), (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
            cv.putText(righthalf, str(rightnonzero), (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
            cv.imshow('Left half binary',lefthalf)
            cv.imshow('Right half binary',righthalf)
            if timer_started==False:
                startt=time.time()
                print('Blink timer started')
                angle=180
                set_data()
                serial_com()
                lfreq=rfreq=0
                timer_started=True
            if startt!=0 and time.time()-startt<=5:
                if leftnonzero>100 and onl==0:
                    onl=1
                if leftnonzero<20 and onl==1:
                    onl=0
                    lfreq+=1
                if rightnonzero>50 and onr==0:
                    onr=1
                if rightnonzero<20 and onr==1:
                    onr=0
                    rfreq+=1
            if startt!=0 and time.time()-startt>5 and printed==0:
                print('Left Frequency: ',lfreq)
                print('Right Frequency: ',rfreq)
                if lfreq>rfreq:
                    pred_turn['left']+=1000
                    print('Go left...')
                else:
                    pred_turn['right']+=1000
                    print('Go right...')
                printed=1
                freq_being_found=False
    cv.imshow('LED Blinking Frequency',ledthresh)
#blink_freq ends


#stop_detection starts
stop_detection_stat=0
stopin=False
stopctr=0

def stop_detection(orig):
    global stop_detection_stat,angle,stopin,stopctr
    if stop_detection_stat==0:
        cv.namedWindow('Stop square')
        cv.createTrackbar('Low H', 'Stop square' ,48 , 179, nothing)
        cv.createTrackbar('High H', 'Stop square' , 63, 179, nothing)
        cv.createTrackbar('Low S', 'Stop square' , 34, 255, nothing)
        cv.createTrackbar('High S', 'Stop square' , 82, 255, nothing)
        cv.createTrackbar('Low V', 'Stop square' , 59, 255, nothing)
        cv.createTrackbar('High V', 'Stop square' , 255, 255, nothing)
        stop_detection_stat=1

    low_H=cv.getTrackbarPos('Low H', 'Stop square')
    low_S=cv.getTrackbarPos('Low S', 'Stop square')
    low_V=cv.getTrackbarPos('Low V', 'Stop square')
    high_H=cv.getTrackbarPos('High H', 'Stop square')
    high_S=cv.getTrackbarPos('High S', 'Stop square')
    high_V=cv.getTrackbarPos('High V', 'Stop square')
    height,width,channels= orig.shape
    orig=roi(orig,'Stop square ROI Selector',139,354,0,width) #357,480,5,205
    orig=cv.GaussianBlur(orig,(5,5),0)
    hsv = cv.cvtColor(orig, cv.COLOR_BGR2HSV)
    stopthresh= cv.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))
    stopthresh=cv.morphologyEx(stopthresh,cv.MORPH_OPEN,(5,5))
    stopthresh=cv.morphologyEx(stopthresh,cv.MORPH_CLOSE,(5,5))
    nonzeropix=cv.countNonZero(stopthresh)
    cv.putText(stopthresh, str(nonzeropix), (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)

    if nonzeropix>9000 and stopin==False:
        print('Stop square enters the frame')
        #cv.putText(stopthresh, 'Stop square enters the frame', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
    	stopin=True

    elif nonzeropix<5000 and stopin==True:
        stopctr+=1
        print('Stop square exits the frame: ',stopctr)
        cv.putText(stopthresh, 'Stop square exits the frame', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
        if stopctr>=15:
            stopctr=0
            angle=180
            cv.putText(stopthresh, 'Time to stop', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),lineType=cv.LINE_AA)
            print('Time to stop')
            stopin=False
            set_data()
            serial_com()
            exit(0)



    else:
        stopctr=0
    cv.imshow('Stop square',stopthresh)




#path detection starts
pred_turn={'left': 0,'right':0}
angle=0
path_detection_stat=0
sec_pred='none'
def path_detection(orig):
    global angle,pred_turn,path_detection_stat,stopin,sec_pred
    #orig = roi(orig,'Path ROI Selector')
    gray = cv.cvtColor(orig, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray,(5,5),0)

    if path_detection_stat==0:
        cv.namedWindow('Path Binary')
        cv.namedWindow('Canny edge')
        cv.createTrackbar('Threshold', 'Path Binary' , 94, 255, nothing)
        cv.createTrackbar('Hough threshold', 'Path Binary' , 50, 200, nothing)
        cv.createTrackbar('Rho', 'Path Binary' , 2, 10, nothing)
        cv.createTrackbar('Min line length', 'Path Binary' , 62, 500, nothing)
        cv.createTrackbar('Lower', 'Canny edge' , 230, 500, nothing)
        cv.createTrackbar('Upper', 'Canny edge' , 280, 500, nothing)
        path_detection_stat=1


    threshold=cv.getTrackbarPos('Threshold','Path Binary')
    houghthresh=cv.getTrackbarPos('Hough threshold','Path Binary')
    rho=cv.getTrackbarPos('Rho','Path Binary')
    lmin=cv.getTrackbarPos('Min line length','Path Binary')
    cannylthresh=cv.getTrackbarPos('Lower','Canny edge')
    cannyhthresh=cv.getTrackbarPos('Upper','Canny edge')


    ret,paththresh = cv.threshold(gray,threshold,255,cv.THRESH_BINARY)
    paththresh=cv.morphologyEx(paththresh,cv.MORPH_OPEN,(5,5))
    paththresh=cv.morphologyEx(paththresh,cv.MORPH_CLOSE,(5,5))

    cv.imshow('Path Binary',paththresh)
    edges=cv.Canny(paththresh,cannylthresh,cannyhthresh)
    cv.imshow('Canny edge',edges)

    linesP = cv.HoughLinesP(edges, rho, np.pi / 180, houghthresh, minLineLength=lmin, maxLineGap= 300)

    if linesP is not None:
        height,width,channels = orig.shape
        perpens=[]
        pmid=[0,0]
        right=left=0
        front=[0,0]
        fmid=[]
        fctr=rctr=lctr=0
        for i in range(len(linesP)):
             l = linesP[i][0]
             if abs(l[2]-l[0]) <=20:
                   perpens.append((round((l[0]+l[2])/2),round((l[1]+l[3])/2)))
                   cv.line(orig, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (255,255,0), 3, cv.LINE_AA)
                   continue
             slope=float(-l[3]+l[1])/float(l[2]-l[0])
             if abs(slope)<0.3:  #front lines
                   cv.line(orig,(int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (255,0,0), 3, cv.LINE_AA)
                   fmid.append(int((l[1]+l[3])/2))
                   front[0]+=slope
                   fctr+=1
             elif -3<slope<0:    #right lines
                   cv.line(orig, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (0,255,0), 3, cv.LINE_AA)
                   right += slope
                   rctr+=1

             elif 0<slope<3:     #left lines
                   cv.line(orig, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (0,0,255), 3, cv.LINE_AA)
                   left += slope
                   lctr+=1

        if len(perpens)!=0:
             for z in perpens:
                   pmid[0]+=z[0]
                   pmid[1]+=z[1]
             pmid[0]/=len(perpens)
             pmid[1]/=len(perpens)



        if fctr!=0:
             front[1]=max(fmid)
             front[0]=front[0]/fctr
        if rctr!=0:
             right=right/rctr
        if lctr!=0:
             left=left/lctr

        if lctr>0 and rctr>0:  #b=3
            if abs(abs(right)-abs(left)-0.026)<0.05:
                angle=0
            elif abs(left)<abs(right)+0.026:
                angle=20
            else:
                angle=-20
            print(math.atan(abs(right)-abs(left)))
            #angle=0

            if pmid[0]!=0:
                if pmid[0]<width/2:
                    pred_turn['left']+=1
                    #print('Left turn predicted')
                else:
                    pred_turn['right']+=1
                    #print('Right turn predicted')

        elif lctr>0 or rctr>0:  #b=2

             # if (lctr>0 and rctr==0 and len(perpens)>=1) or fctr==0:
             #    angle=45
             #
             # elif lctr>0 and rctr==0:
             #    angle=20
             #
             # elif rctr>0 and lctr==0 and len(perpens)>=1 or fctr==0:
             #    angle=-45
             #
             # elif rctr>0 and lctr==0:
             #    angle=-20
             print('b=2')
             if lctr>0 and rctr==0:
                 print(left)
                 angle=20
                 sec_pred='right'


             elif rctr>0 and lctr==0:
                 print(right)
                 angle=-20
                 sec_pred='left'


             if pmid[0]!=0:
                 if pmid[0]<width/2:
                     pred_turn['left']+=1
                     #print('Left turn predicted')
                 else:
                     pred_turn['right']+=1
                     #print('Right turn predicted')


        elif fctr>0 and (rctr==0 and lctr==0): #b=1
            print('Ratio: ',float(front[1])/height)
            print('Slope of front line: ',front[0])
            if float(front[1])/height> 0.72 and stopin==False:
                if pred_turn['right']>pred_turn['left']:
                    angle= 90 #sharp right turn

                elif pred_turn['right']<pred_turn['left']:
                    angle= -90 #sharp left turn

                elif pred_turn['right']==pred_turn['left']:
                    if sec_pred=='left':
                        angle=-90
                    elif sec_pred=='right':
                        angle=90



            elif float(front[1])/height> 0.55 and stopin==False:
                if pred_turn['right']>pred_turn['left']:
                    angle= 85 #sharp right turn

                elif pred_turn['right']<pred_turn['left']:
                    angle= -85 #sharp left turn

                elif pred_turn['right']==0 and pred_turn['left']==0:
                    print('random turn ')
                    # angle=60



                else:
                    angle=0
                pred_turn['right']=0
                pred_turn['left']=0
            else:

                if abs(front[0])<0.03:
                    angle=0 #forward
                # elif front[0]<0 and len(perpens)>=1:
                #     angle=45
                elif front[0]<0:
                    angle=45
                # elif front[0]>0 and len(perpens)>=1:
                #     angle=-45
                elif front[0]>0:
                    angle=-45



    else:
        angle=255

#path detection ends

#set_data starts
def set_data():
    global angle, ledmode, dir
    dir='S'
    if ledmode=='T':
        dir='T'
        ledmode='F'

    elif angle==255:
        dir='m'
        print('Special manouevre')

    elif angle==85:
        dir='fr'

    elif angle==-85:
        dir='fl'

    elif angle==90:
        dir='r'
        print('Sharp right')

    elif angle==-90:
        dir='l'
        print('Sharp left')

    elif angle==45:
        dir='R'
        print('Forward right')

    elif angle==-45:
        dir='L'
        print('Forward left')

    elif angle==0:
        dir='F'
        print('Forward')

    elif angle==180:
        dir='S'
        print('Halted')

    elif angle==20:
        dir='D'

    elif angle==-20:
        dir='A'

    elif angle==60:
        dir='B'

#set_data ends

#serial_com starts
ser = serial.Serial('/dev/ttyUSB0',9600)
def serial_com():
    global ser,dir
    time.sleep(0.025)
    if dir=='r' or dir=='l':
        print('only sharp')
        ser.write('S')
        time.sleep(1)
        ser.write('F')
        time.sleep(0.2)
        ser.write(dir)
        time.sleep(0.55)
        ser.write('S')
        time.sleep(1)

    if dir=='fr' or dir=='fl':
        print('forward and sharp')
        ser.write('S')
        time.sleep(1)
        ser.write('F')
        time.sleep(0.3)
        ser.write(dir)
        time.sleep(0.45)
        ser.write('S')
        time.sleep(1)

    # elif dir=='B':
    #     print('Going backwards')
    #     ser.write('B')
    #     time.sleep(0.5)

    elif dir=='T':
        print('Writing T')
        ser.write('T')

    else:
        print("Sending: ",dir)
        ser.write(dir)



#serial_com ends


#main body starts
cap = cv.VideoCapture(1)
if(cap.isOpened()==False):    print("Unable to detect webcam...")
else:
    cv.namedWindow('Video Capture')
    while(cv.waitKey(25) & 0xFF != ord('q')):
    	ret, orig = cap.read()
        if freq_being_found==False:
            stop_detection(orig)
            led_detection(orig)
            path_detection(orig)
            set_data()
            serial_com()
        # if freq_count==0:
        #     blink_freq(orig)
        cv.imshow('Video Capture',orig)


cap.release()
cv.destroyAllWindows()
#main body ends
