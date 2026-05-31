"""
monitor.py — RoboVerse 2026 Live Diagnostic HUD
Run in a SECOND terminal alongside main.py.
Usage: python3 monitor.py
"""
import json, os, time, threading
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import config as C

HUD_W = 360; HUD_H = 480
gz_node = None
latest_frame = None
frame_lock = threading.Lock()
frame_count = 0
status = {}
status_lock = threading.Lock()

def camera_cb(msg: Image):
    global latest_frame, frame_count
    try:
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        frame = raw.reshape((msg.height, msg.width, 3))
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with frame_lock:
            latest_frame = bgr
            frame_count += 1
    except Exception as e:
        print(f"[Monitor] Camera error: {e}")

def status_reader():
    global status
    while True:
        try:
            if os.path.exists("nav_status.json"):
                with open("nav_status.json") as f:
                    data = json.load(f)
                with status_lock:
                    status = data
        except: pass
        time.sleep(0.3)

def draw_hud(st):
    p = np.zeros((HUD_H, HUD_W, 3), dtype=np.uint8); p[:] = (20,20,20)
    def t(s,x,y,c=(200,200,200),sc=0.45,b=False):
        cv2.putText(p,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,sc,c,2 if b else 1,cv2.LINE_AA)
    def hl(y): cv2.line(p,(8,y),(HUD_W-8,y),(50,50,50),1)

    cv2.rectangle(p,(0,0),(HUD_W,34),(30,30,30),-1)
    t("DRONE MONITOR",8,24,(100,220,220),0.55,True)
    t(time.strftime("%H:%M:%S"),HUD_W-78,24,(90,90,90),0.4)
    action=st.get("action","—")
    ac={"GOAL":(80,200,80),"CORRIDOR":(200,200,60),"NEAR_WALL":(60,180,240),
        "OBSTACLE":(60,80,240),"BACK_OFF":(0,60,200),"REVERSE":(0,0,200),
        "TURN_TO_GOAL":(180,180,60)}.get(action,(180,180,180))
    cv2.rectangle(p,(8,42),(HUD_W-8,68),(35,35,35),-1)
    t(action,14,60,ac,0.55,True)
    hl(74)
    t("POSITION",8,90,(100,100,100),0.35,True)
    t(f"N:{st.get('north',0):+.1f} E:{st.get('east',0):+.1f}",14,108)
    t(f"Alt:{st.get('alt',0):.1f}m Yaw:{st.get('yaw',0):.0f}",14,126)
    hl(134)
    t("DEPTH",8,150,(100,100,100),0.35,True)
    l=st.get("left",0);c=st.get("center",0);r=st.get("right",0)
    def sc(v): return (80,80,240) if v<1.0 else (60,180,240) if v<2.5 else (80,200,80)
    t(f"L:{l:.1f}",14,168,sc(l));t(f"C:{c:.1f}",120,168,sc(c));t(f"R:{r:.1f}",226,168,sc(r))
    for i,(v,cl) in enumerate([(l,sc(l)),(c,sc(c)),(r,sc(r))]):
        bx=14+i*106;cv2.rectangle(p,(bx,175),(bx+90,181),(50,50,50),-1)
        f=int(90*min(1,v/10));
        if f>0: cv2.rectangle(p,(bx,175),(bx+f,181),cl,-1)
    hl(190)
    t("GOAL",8,206,(100,100,100),0.35,True)
    t(f"({st.get('goal_n',0):.0f},{st.get('goal_e',0):.0f}) d={st.get('goal_dist',0):.0f}m",14,222)
    hl(232)
    cov=st.get("coverage",0)
    t("COVERAGE",8,248,(100,100,100),0.35,True);t(f"{cov:.0f}%",90,248,(80,200,80),0.45,True)
    cv2.rectangle(p,(14,256),(HUD_W-14,264),(50,50,50),-1)
    f=int((HUD_W-28)*cov/100)
    if f>0: cv2.rectangle(p,(14,256),(14+f,264),(80,200,80),-1)
    hl(274)
    mt=st.get("mission_t",0);bonus=max(0,300-int(mt))
    mm,ss=divmod(int(mt),60);bm,bs=divmod(bonus,60)
    bc=(80,200,80) if bonus>60 else (60,180,240) if bonus>0 else (80,80,240)
    t(f"TIME {mm:02d}:{ss:02d}",14,292);t(f"BONUS {bm:02d}:{bs:02d}",180,292,bc)
    hl(304)
    yc=st.get("yellow",0);rc=st.get("red",0);sc2=st.get("score",0)
    t(f"Yellow:{yc}",14,322,(50,220,220));t(f"Red:{rc}",140,322,(80,80,240))
    t(f"SCORE:{sc2}",14,346,(80,200,80) if sc2>0 else (120,120,120),0.55,True)
    q=yc>=1 and rc>=1
    t("QUALIFIES" if q else "NOT YET",200,346,(80,200,80) if q else (80,80,240),0.4,True)
    hl(360);t(f"Cam frames: {frame_count}",14,378,(80,80,80),0.35)
    return p

def main():
    global gz_node
    print("="*50);print("  RoboVerse 2026 — Monitor HUD");print("="*50)
    gz_node=Node()
    ok=gz_node.subscribe(Image,C.IMAGE_TOPIC,camera_cb)
    print(f"  Camera: {C.IMAGE_TOPIC}" if ok else "  WARN: Camera subscribe failed")
    threading.Thread(target=status_reader,daemon=True).start()
    print("  Status reader started\n  Press Q/ESC to quit\n")
    cv2.namedWindow("RoboVerse Monitor",cv2.WINDOW_AUTOSIZE)
    while True:
        with frame_lock:
            raw_frame=latest_frame.copy() if latest_frame is not None else None
        # RESIZE camera to match HUD height (camera may be 1080p, 720p, etc.)
        if raw_frame is not None:
            h,w=raw_frame.shape[:2]
            scale=HUD_H/h
            cam_panel=cv2.resize(raw_frame,(int(w*scale),HUD_H))
        else:
            cam_panel=np.zeros((HUD_H,640,3),dtype=np.uint8)
            cv2.putText(cam_panel,"Waiting for camera...",(120,220),
                        cv2.FONT_HERSHEY_SIMPLEX,0.8,(80,80,80),2)
        with status_lock:
            st=dict(status)
        hud_panel=draw_hud(st)
        combined=np.hstack([cam_panel,hud_panel])
        cv2.imshow("RoboVerse Monitor",combined)
        if cv2.waitKey(33)&0xFF in (ord('q'),ord('Q'),27): break
    cv2.destroyAllWindows();print("Monitor closed.")

if __name__=="__main__":
    main()
