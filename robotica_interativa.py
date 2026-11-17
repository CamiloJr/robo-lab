"""ROBO LAB - simulador de IA na robotica, para a disciplina Introducao a IA.

Requer apenas pygame-ce. Todos os algoritmos sao locais e explicaveis.
Executar: python robotica_interativa.py
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from datetime import datetime
from functools import lru_cache

try:
    import pygame
except ImportError:
    raise SystemExit('Dependencia ausente: execute INICIAR_WINDOWS.bat ou pip install pygame-ce')

from navegacao import (
    Simulator, WIDTH, HEIGHT, CELL, COLS, ROWS, ROBOT_R,
    Grid, occupied, dist, norm_angle, clamp,
)

VIRTUAL_W, VIRTUAL_H = 1536, 920
FPS = 55
MAP = pygame.Rect(28, 108, 1080, 670)
SIDEX, SIDEW = 1132, 376

BG = (8, 15, 28)
CARD = (17, 29, 47)
CARD2 = (20, 38, 60)
BORDER = (46, 69, 91)
MUTED = (133, 157, 177)
WHITE = (235, 244, 251)
CYAN = (57, 213, 245)
BLUE = (57, 126, 252)
LIME = (112, 240, 164)
ORANGE = (255, 185, 85)
PURPLE = (188, 135, 255)
RED = (255, 109, 125)


@lru_cache(maxsize=64)
def font(size, bold=False):
    name = pygame.font.match_font('segoeui') or pygame.font.match_font('arial')
    obj=pygame.font.Font(name, size)
    obj.set_bold(bold)
    return obj


def text(surf, string, x, y, size=16, color=WHITE, bold=False, anchor='topleft', alpha=255):
    img = font(size, bold).render(str(string), True, color)
    if alpha < 255:
        img.set_alpha(alpha)
    rect = img.get_rect(**{anchor:(round(x), round(y))})
    surf.blit(img, rect)
    return rect


def trunc(s, max_chars=48):
    return s if len(s) <= max_chars else s[:max_chars-1]+'…'


def rounded(surf, rect, color, radius=12, border=None, border_w=1):
    pygame.draw.rect(surf, color, rect, border_radius=radius)
    if border:
        pygame.draw.rect(surf, border, rect, border_w, border_radius=radius)


def glow_circle(surf, color, center, radius, alpha=50):
    r = int(radius)
    layer = pygame.Surface((2*r+4,2*r+4), pygame.SRCALPHA)
    pygame.draw.circle(layer, (*color, alpha//5), (r+2,r+2),r)
    pygame.draw.circle(layer, (*color, alpha//3), (r+2,r+2),int(r*.70))
    pygame.draw.circle(layer, (*color, alpha), (r+2,r+2),int(r*.42))
    surf.blit(layer,(center[0]-r-2,center[1]-r-2))


def draw_dashed(surf, color, p1, p2, width=2, dash=9, gap=7, phase=0):
    dx=p2[0]-p1[0]
    dy=p2[1]-p1[1]
    length=math.hypot(dx,dy)
    if length<1:
        return
    ux,uy=dx/length,dy/length
    pos=-phase % (dash+gap)
    while pos<length:
        a=max(0,pos)
        b=min(length,pos+dash)
        if b>a:
            pygame.draw.line(surf,color,(int(p1[0]+ux*a),int(p1[1]+uy*a)),
                             (int(p1[0]+ux*b),int(p1[1]+uy*b)),width)
        pos+=dash+gap


def rotate_pos(x,y,theta,localx,localy):
    c,s=math.cos(theta),math.sin(theta)
    return int(x+localx*c-localy*s),int(y+localx*s+localy*c)


def wrap_text(value, line_width=44, max_lines=3):
    words=value.split()
    lines=[]
    cur=''
    for w in words:
        if len(cur)+len(w)+1>line_width and cur:
            lines.append(cur)
            cur=w
        else:
            cur=(cur+' '+w).strip()
    if cur:lines.append(cur)
    return lines[:max_lines]


class RoboLab:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption('ROBO LAB | Inteligencia Artificial na Robotica - Aula 7')
        self.screen=pygame.display.set_mode((VIRTUAL_W,VIRTUAL_H),pygame.RESIZABLE)
        self.canvas=pygame.Surface((VIRTUAL_W,VIRTUAL_H)).convert()
        self.clock=pygame.time.Clock()
        self.sim=Simulator()
        self.running=True
        self.map_edit='goal'
        self.buttons=[]
        self.help_open=False
        self.last_save=''
        self.manual=(0,0)
        self.ui_time=0.0
        self.celebration=0.0
        self.last_reached=False
        self.map_background=self.make_map_base()
        self.graph_surf=None
        self.frame_counter=0
        self.set_message('Defina um destino clicando no mapa. A* calcula a rota automaticamente.')

    def set_message(self,message):
        self.sim.message=message

    def coordinates(self,event_pos):
        w,h=self.screen.get_size()
        scale=min(w/VIRTUAL_W,h/VIRTUAL_H)
        offx=(w-VIRTUAL_W*scale)*.5
        offy=(h-VIRTUAL_H*scale)*.5
        return ((event_pos[0]-offx)/scale,(event_pos[1]-offy)/scale)

    def world_pos(self,p):
        if MAP.collidepoint(p):
            return ((p[0]-MAP.x)*WIDTH/MAP.w,(p[1]-MAP.y)*HEIGHT/MAP.h)
        return None

    def mouse_click(self,point,button):
        if self.help_open:
            if button==1:self.help_open=False
            return
        if button==1:
            for box,action in reversed(self.buttons):
                if box.collidepoint(point):
                    action()
                    return
        p=self.world_pos(point)
        if p:
            if button==3 or self.map_edit=='remove':
                self.sim.remove_obstacle(*p)
            elif self.map_edit=='add':
                self.sim.add_obstacle(*p)
            elif self.map_edit=='goal':
                self.sim.set_goal(*p)

    def create_button(self,surf,label,rect,callback,active=False,kind='plain',fontsize=14):
        # Os controles do layout podem passar (x, y, largura, altura) ou pygame.Rect.
        # Normalize antes de usar atributos .centerx/.centery e de testar o clique.
        rect = pygame.Rect(rect)
        mouse=self.coordinates(pygame.mouse.get_pos())
        hover=rect.collidepoint(mouse)
        if active:
            fill=(34, 94, 119)
            edge=CYAN
            txt=(221,250,253)
        elif kind=='danger':
            fill=(75,45,58) if hover else (55,35,48)
            edge=(142,71,88)
            txt=(254, 194, 201)
        else:
            fill=(32,54,76) if hover else (23,42,61)
            edge=(68,110,139) if hover else BORDER
            txt=WHITE if hover else MUTED
        rounded(surf,rect,fill,9,edge)
        text(surf,label,rect.centerx,rect.centery,fontsize,txt,active or hover,anchor='center')
        self.buttons.append((rect,callback))

    def toggle(self,name,label=None):
        setattr(self.sim,name,not getattr(self.sim,name))
        if name=='dynamic_enabled':
            self.sim.plan_route(force=True)
        self.set_message(label or '{}: {}'.format(name, 'ATIVO' if getattr(self.sim,name) else 'INATIVO'))

    def adjust(self,name,step,low,high):
        setattr(self.sim,name,clamp(getattr(self.sim,name)+step,low,high))
        self.sim.update_lidar()
        self.sim.last_dwa_t=-100

    def make_map_base(self):
        surf=pygame.Surface((WIDTH,HEIGHT)).convert()
        surf.fill((11,24,40))
        for x in range(0,WIDTH+1,CELL):
            col=(30,56,71) if x%100==0 else (21,42,57)
            pygame.draw.line(surf,col,(x,0),(x,HEIGHT),1)
        for y in range(0,HEIGHT+1,CELL):
            col=(30,56,71) if y%100==0 else (21,42,57)
            pygame.draw.line(surf,col,(0,y),(WIDTH,y),1)
        for x in range(0,WIDTH,100):
            for y in range(0,HEIGHT,100):
                pygame.draw.circle(surf,(48,81,91),(x,y),1)
        # Safety/perimeter frame
        pygame.draw.rect(surf,(43,83,102),(1,1,WIDTH-2,HEIGHT-2),2,border_radius=3)
        return surf

    def map_surface(self):
        s=self.sim
        surf=self.map_background.copy()
        # A* cells explored, visualizing heuristic search
        if s.show_search:
            layer=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
            for gx,gy in s.visited:
                pygame.draw.rect(layer,(247,176,66,34),(gx*CELL+1,gy*CELL+1,CELL-2,CELL-2),border_radius=3)
            surf.blit(layer,(0,0))
        # Q-learning value map
        if s.show_heat and s.qlearn.episodes>0:
            heat=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
            for cell,qs in s.qlearn.q.items():
                if cell in s.grid_static.blocked:continue
                peak=max(qs)
                if peak>0:
                    alpha=int(clamp(peak/140*95,5,95))
                    pygame.draw.rect(heat,(*PURPLE,alpha),(cell[0]*CELL+2,cell[1]*CELL+2,CELL-4,CELL-4),border_radius=2)
            surf.blit(heat,(0,0))
        # World markings
        if s.scenario=='armazem':
            for y in (225,423):
                for x in range(40,960,42):
                    pygame.draw.line(surf,(52,69,73),(x,y),(x+19,y),2)
        if s.scenario=='cidade':
            for y in (270,316):
                for x in range(20,980,50):
                    pygame.draw.line(surf,(69,81,74),(x,y),(x+22,y),2)
            for x in (375,632):
                for y in range(20,600,51):
                    pygame.draw.line(surf,(64,76,78),(x,y),(x,y+24),2)
        # Static obstacles
        for o in s.obstacles:
            if o.is_circle:
                x,y,r=int(o.x),int(o.y),int(o.r)
                pygame.draw.circle(surf,(5,13,21),(x+5,y+7),r+2)
                pygame.draw.circle(surf,(60,78,90),(x,y),r)
                pygame.draw.circle(surf,(84,121,138),(x,y),r,2)
                pygame.draw.circle(surf,(26,47,65),(x,y),max(6,r-8))
                for a in range(0,360,60):
                    v=math.radians(a)
                    pygame.draw.circle(surf,(118,147,146),(int(x+math.cos(v)*(r-7)),int(y+math.sin(v)*(r-7))),2)
            else:
                rr=pygame.Rect(round(o.x),round(o.y),round(o.w),round(o.h))
                pygame.draw.rect(surf,(6,14,24),rr.move(5,6),border_radius=7)
                if o.kind=='user':
                    pygame.draw.rect(surf,(101,68,48),rr,border_radius=6)
                    pygame.draw.rect(surf,ORANGE,rr,2,border_radius=6)
                    pygame.draw.line(surf,(178,125,77),rr.topleft,rr.bottomright,2)
                    pygame.draw.line(surf,(178,125,77),rr.topright,rr.bottomleft,2)
                else:
                    pygame.draw.rect(surf,(37,66,84),rr,border_radius=7)
                    pygame.draw.rect(surf,(92,120,142),rr,2,border_radius=7)
                    for yy in range(rr.y+17,rr.bottom-6,18):
                        pygame.draw.line(surf,(54,95,111),(rr.x+11,yy),(rr.right-11,yy),2)
                    text(surf,'ZONA',rr.centerx,rr.centery-6,10,(101,139,152),True,'center')
        # Robot travelled trail
        if len(s.trail)>1:
            pts=[(int(px),int(py)) for px,py in s.trail]
            pygame.draw.lines(surf,(28,91,114),False,pts,7)
            pygame.draw.lines(surf,(59,160,185),False,pts,2)
        # Learned policy route in contrasting purple
        if s.show_q:
            route=s.q_path()
            if len(route)>1:
                pts=[Grid.center(c) for c in route]
                for a,b in zip(pts,pts[1:]):
                    draw_dashed(surf,(197,140,254),a,b,3,7,7,s.time*8)
                for p in pts[::max(1,len(pts)//25)]:
                    pygame.draw.circle(surf,PURPLE,(int(p[0]),int(p[1])),2)
        # Planned A* path
        if s.show_path and len(s.path)>1:
            coords=[(int(x),int(y)) for x,y in s.path]
            pygame.draw.lines(surf,(19,91,126),False,coords,13)
            pygame.draw.lines(surf,(48,180,217),False,coords,4)
            for a,b in zip(coords,coords[1:]):
                draw_dashed(surf,(158,245,255),a,b,2,13,15,s.time*45)
            for waypoint in s.path[1:-1]:
                pygame.draw.circle(surf,(23,42,57),(int(waypoint[0]),int(waypoint[1])),7)
                pygame.draw.circle(surf,CYAN,(int(waypoint[0]),int(waypoint[1])),5,2)
        # DWA candidate trajectories
        if s.autopilot and s.dwa:
            candidates=s.dwa[1:19:2]
            traj_layer=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
            for c in candidates:
                pygame.draw.lines(traj_layer,(218,169,90,51),False,[(int(px),int(py)) for px,py in c.positions],2)
            surf.blit(traj_layer,(0,0))
            if s.chosen:
                coords=[(int(x),int(y)) for x,y in s.chosen.positions]
                if len(coords)>1:
                    pygame.draw.lines(surf,(38,117,92),False,coords,10)
                    pygame.draw.lines(surf,LIME,False,coords,3)
                px,py=coords[-1]
                pygame.draw.circle(surf,LIME,(px,py),5)
        # Moveable agents
        if s.dynamic_enabled:
            for a in s.actors:
                x,y=int(a.x),int(a.y)
                glow_circle(surf,ORANGE,(x,y),42,56)
                pygame.draw.circle(surf,(9,20,28),(x+3,y+4),int(a.radius+4))
                pygame.draw.circle(surf,(164,102,57),(x,y),int(a.radius))
                pygame.draw.circle(surf,ORANGE,(x,y),int(a.radius),2)
                vx,vy=a.velocity()
                ang=math.atan2(vy,vx)
                front=rotate_pos(x,y,ang,12,0)
                pygame.draw.line(surf,(253,239,195),(x,y),front,3)
                pygame.draw.circle(surf,(252,231,176),front,3)
                text(surf,'AGV',x,y-31,11,ORANGE,True,'midbottom')
        # LiDAR beams on upper half with visible range cutoff
        if s.show_lidar and s.scan:
            scanlayer=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
            x,y=s.x,s.y
            for i,(a,rng) in enumerate(s.scan):
                end=(x+math.cos(a)*rng,y+math.sin(a)*rng)
                strength=(rng < s.lidar_range-.2)
                col=(255,174,81,74) if rng<70 else (69,216,230,40 if strength else 22)
                pygame.draw.line(scanlayer,col,(int(x),int(y)),(int(end[0]),int(end[1])),1)
                if strength:
                    pygame.draw.circle(scanlayer,(105,229,233,148),(int(end[0]),int(end[1])),2)
            pygame.draw.circle(scanlayer,(48,205,231,36),(int(x),int(y)),s.lidar_range,1)
            surf.blit(scanlayer,(0,0))
        # Destination marker, with pulse
        gx,gy=map(int,s.goal)
        glow_circle(surf,LIME,(gx,gy),48,80)
        pulse=int(22 + 7*(1+math.sin(s.time*3)))
        pygame.draw.circle(surf,LIME,(gx,gy),pulse,2)
        pygame.draw.circle(surf,(16,58,51),(gx,gy),15)
        pygame.draw.circle(surf,LIME,(gx,gy),12,2)
        pygame.draw.circle(surf,LIME,(gx,gy),4)
        text(surf,'DESTINO',gx,gy-40,12,LIME,True,'midbottom')
        # Robot itself
        self.robot(surf,s.x,s.y,s.theta)
        # HUD in map
        rounded(surf,(18,17,370,64),(11,28,43),10,(41,77,97))
        scene_names={'armazem':'CENTRO LOGISTICO','resgate':'OPERACAO RESGATE','cidade':'MOBILIDADE URBANA'}
        text(surf,'AMBIENTE / {}'.format(scene_names[s.scenario]),33,30,14,WHITE,True)
        text(surf,'PLANEJAMENTO  •  PERCEPCAO  •  CONTROLE',33,54,12,MUTED)
        if s.reached:
            rounded(surf,(280,220,430,130),(16,64,56),18,LIME,3)
            text(surf,'MISSAO CONCLUIDA',495,252,30,LIME,True,'center')
            text(surf,'Destino alcancado sem colisao.',495,290,18,WHITE,False,'center')
            text(surf,'Clique em outro ponto para uma nova missao.',495,325,15,MUTED,False,'center')
        # Pipeline pedagógico: quatro etapas do ciclo percepção-planejamento-ação.
        rounded(surf,(422,18,542,62),(12,30,45),10,(43,78,94))
        steps=('01  PERCEBER','02  LOCALIZAR','03  PLANEJAR','04  AGIR')
        active=int(s.time*2.1)%4
        for idx,caption in enumerate(steps):
            left=432+idx*133
            pygame.draw.rect(surf,(25,79,86) if idx==active else (28,43,57),
                             (left,28,126,42),border_radius=6)
            text(surf,caption,left+63,49,11,LIME if idx==active else MUTED,True,'center')
        # Scale indicator
        text(surf,'100 unidades',WIDTH-25,HEIGHT-40,12,MUTED,False,'topright')
        pygame.draw.line(surf,(139,161,169),(WIDTH-171,HEIGHT-21),(WIDTH-25,HEIGHT-21),2)
        pygame.draw.line(surf,(139,161,169),(WIDTH-171,HEIGHT-25),(WIDTH-171,HEIGHT-17),2)
        pygame.draw.line(surf,(139,161,169),(WIDTH-25,HEIGHT-25),(WIDTH-25,HEIGHT-17),2)
        return surf

    def robot(self,surf,x,y,theta):
        x,y=round(x),round(y)
        glow_circle(surf,CYAN,(x,y),60,97)
        # Dynamic wave around perceived pose
        pygame.draw.circle(surf,(43,119,131),(x,y),28,1)
        perp=theta+math.pi/2
        for sign in (-1,1):
            center=rotate_pos(x,y,theta,0,sign*14)
            wheel=(rotate_pos(x,y,theta,-11,sign*14),rotate_pos(x,y,theta,11,sign*14))
            pygame.draw.line(surf,(11,24,39),wheel[0],wheel[1],7)
            pygame.draw.line(surf,(92,140,158),wheel[0],wheel[1],2)
        pygame.draw.circle(surf,(8,17,30),(x,y),24)
        pygame.draw.circle(surf,(27,84,100),(x,y),20)
        pygame.draw.circle(surf,CYAN,(x,y),20,3)
        front=rotate_pos(x,y,theta,17,0)
        back_left=rotate_pos(x,y,theta,-9,-9)
        back_right=rotate_pos(x,y,theta,-9,9)
        pygame.draw.polygon(surf,(178,246,253),(front,back_left,back_right))
        pygame.draw.circle(surf,(5,28,43),(x,y),7)
        pygame.draw.circle(surf,LIME,(x,y),4)
        text(surf,'ROBO / IA',x,y-37,12,CYAN,True,'midbottom')

    def header(self,surf):
        pygame.draw.rect(surf,(12,24,39),(0,0,VIRTUAL_W,91))
        pygame.draw.line(surf,BORDER,(0,90),(VIRTUAL_W,90),1)
        glow_circle(surf,CYAN,(45,45),31,42)
        rounded(surf,(25,25,41,41),(15,58,78),11,CYAN,2)
        pygame.draw.rect(surf,CYAN,(33,38,25,9),border_radius=4)
        pygame.draw.circle(surf,(13,30,41),(40,42),2)
        pygame.draw.circle(surf,(13,30,41),(52,42),2)
        text(surf,'ROBO',79,14,28,WHITE,True)
        text(surf,'LAB',153,14,28,CYAN,True)
        text(surf,'IA APLICADA A ROBOTICA  /  SIMULADOR INTERATIVO',80,53,12,MUTED)
        rounded(surf,(498,28,205,34),(24,41,55),17,(49,84,96))
        pygame.draw.circle(surf,LIME,(519,45),4)
        text(surf,'PROCESSAMENTO LOCAL',535,35,13,WHITE)
        for label,name,x,width in [('ARMAZEM','armazem',730,108),('RESGATE','resgate',845,100),('CIDADE','cidade',953,90)]:
            self.create_button(surf,label,(x,28,width,37),lambda n=name:self.change_scene(n),self.sim.scenario==name,fontsize=13)
        self.create_button(surf,'REINICIAR',(1057,28,108,37),lambda:self.change_scene(self.sim.scenario),fontsize=13)
        self.create_button(surf,'RETOMAR' if self.sim.is_paused else 'PAUSAR',(1172,28,97,37),self.pause, self.sim.is_paused,fontsize=13)
        self.create_button(surf,'AJUDA  ?', (1278,28,98,37),self.toggle_help,fontsize=13)
        rounded(surf,(1391,28,119,37),(29,45,61),9,BORDER)
        text(surf,'AULA 07',1451,46,16,CYAN,True,'center')

    def change_scene(self,name):
        self.sim.reset(name)
        self.set_message('Novo ambiente: escolha o destino ou acompanhe o robo autonomo.')

    def pause(self):
        self.sim.is_paused=not self.sim.is_paused

    def toggle_help(self):
        self.help_open=not self.help_open

    def side_card(self,surf,y,height,title,tag=None):
        rect=(SIDEX,y,SIDEW,height)
        rounded(surf,rect,CARD,12,BORDER)
        text(surf,title,SIDEX+17,y+14,15,WHITE,True)
        if tag:
            text(surf,tag,SIDEX+SIDEW-17,y+16,11,CYAN,True,'topright')
        pygame.draw.line(surf,(41,63,80),(SIDEX+16,y+41),(SIDEX+SIDEW-16,y+41))
        return SIDEX+16

    def sidebar(self,surf):
        s=self.sim
        # control
        rounded(surf,(SIDEX,108,SIDEW,64),CARD,12,BORDER)
        text(surf,'MODO',SIDEX+16,120,12,MUTED,True)
        self.create_button(surf,'AUTONOMO',(SIDEX+79,118,130,42),lambda:self.set_auto(True),s.autopilot,fontsize=14)
        self.create_button(surf,'MANUAL',(SIDEX+216,118,142,42),lambda:self.set_auto(False),not s.autopilot,fontsize=14)
        # telemetry
        x=self.side_card(surf,182,133,'TELEMETRIA DO ROBO','CINEMATICA')
        text(surf,'VELOCIDADE ATUAL',x,237,12,MUTED,True)
        text(surf,str(int(abs(s.v))).zfill(3),x,253,38,CYAN,True)
        text(surf,'unid./s',x+92,276,12,MUTED)
        rounded(surf,(SIDEX+192,244,1,56),(44,65,78),0)
        text(surf,'ORIENTACAO',SIDEX+211,238,12,MUTED,True)
        text(surf,'{}°'.format(int(math.degrees(s.theta))%360),SIDEX+211,259,29,WHITE,True)
        text(surf,'V. max: {} unid./s'.format(s.max_speed),x,297,11,MUTED)
        self.create_button(surf,'−',(SIDEX+278,287,32,26),lambda:self.adjust('max_speed',-25,50,225),fontsize=15)
        self.create_button(surf,'+',(SIDEX+316,287,32,26),lambda:self.adjust('max_speed',25,50,225),fontsize=15)
        # Sensors
        x=self.side_card(surf,325,136,'SENSORES VIRTUAIS','LiDAR 270°')
        text(surf,'72 feixes  |  atualizacao ~10 Hz',x,370,12,MUTED)
        near=(s.min_lidar if s.scan else s.lidar_range)
        text(surf,'MENOR DISTANCIA',x,398,12,MUTED,True)
        text(surf,'{:03d}'.format(round(near)),x+189,391,29,ORANGE if near<70 else LIME,True)
        text(surf,'u',x+247,403,13,MUTED)
        rounded(surf,(x,435,170,7),(35,57,70),4)
        fill=int(clamp(near/s.lidar_range,0,1)*170)
        if fill>1:rounded(surf,(x,435,fill,7),ORANGE if near<70 else CYAN,4)
        self.create_button(surf,'-',(SIDEX+275,385,33,31),lambda:self.adjust('lidar_range',-25,100,300),fontsize=18)
        self.create_button(surf,'+',(SIDEX+315,385,33,31),lambda:self.adjust('lidar_range',25,100,300),fontsize=18)
        text(surf,'Alcance: {}'.format(s.lidar_range),SIDEX+272,423,11,MUTED)
        # planning
        x=self.side_card(surf,471,143,'IA: PLANEJAMENTO + CONTROLE','A* / DWA')
        text(surf,'PLANEJADOR A*',x,518,13,CYAN,True)
        text(surf,'{} celulas visitadas'.format(len(s.visited)),x,537,12,MUTED)
        text(surf,'{} replanejamentos'.format(s.replans),x,554,12,WHITE)
        pygame.draw.line(surf,(43,70,90),(SIDEX+189,519),(SIDEX+189,593))
        text(surf,'CONTROLE DWA',SIDEX+210,518,13,LIME,True)
        text(surf,'{} trajetorias validas'.format(len(s.dwa)),SIDEX+210,537,12,MUTED)
        if s.chosen:
            text(surf,'v={:.0f}  ω={:.1f}'.format(s.chosen.v,s.chosen.w),SIDEX+210,555,12,WHITE)
            text(surf,'score {:.2f}'.format(s.chosen.score),SIDEX+210,574,12,MUTED)
        else:
            text(surf,'sem decisao local',SIDEX+210,555,12,MUTED)
        text(surf,'Margem de seguranca: {}'.format(int(s.safety)),x,590,11,MUTED)
        self.create_button(surf,'−',(SIDEX+273,579,35,28),lambda:self.adjust('safety',-3,2,26),fontsize=17)
        self.create_button(surf,'+',(SIDEX+316,579,35,28),lambda:self.adjust('safety',3,2,26),fontsize=17)
        # Q learning
        x=self.side_card(surf,624,183,'APRENDIZADO POR REFORCO','Q-LEARNING')
        ep=s.qlearn.episodes
        self.create_button(surf,'PAUSAR TREINO' if s.learning else ('REINICIAR TREINO' if s.qlearn.finished else 'TREINAR IA'),
                           (x,675,169,35),s.toggle_train,active=s.learning,fontsize=13)
        text(surf,'{} / {} episodios'.format(ep,s.qlearn.target_episodes),SIDEX+201,686,12,MUTED)
        text(surf,'SUCESSO (ULTIMOS 100)',x,719,11,MUTED,True)
        text(surf,'{:.0f}%'.format(s.qlearn.success_rate),SIDEX+344,713,25,PURPLE,True,'topright')
        gx,gy,gw,gh=x,746,344,45
        pygame.draw.rect(surf,(11,25,40),(gx,gy,gw,gh),border_radius=4)
        for frac in (.25,.5,.75):
            yy=int(gy+gh*(1-frac))
            pygame.draw.line(surf,(32,52,69),(gx,yy),(gx+gw,yy))
        points=[]
        for episode,rate in s.qlearn.history:
            xx=gx+int(episode/s.qlearn.target_episodes*gw)
            yy=gy+gh-int(rate*gh)
            points.append((xx,yy))
        if len(points)>1:
            pygame.draw.lines(surf,PURPLE,False,points,2)
            pygame.draw.circle(surf,PURPLE,points[-1],3)
        rounded(surf,(gx,796,int(gw*ep/max(1,s.qlearn.target_episodes)),3),PURPLE,2)
        # diagnostics card
        x=self.side_card(surf,817,74,'DECISAO DO AGENTE','EXPLICAVEL')
        for i,line in enumerate(wrap_text(s.message,54,2)):
            text(surf,line,x,863+i*15,12,LIME if s.reached else WHITE)

    def set_auto(self,value):
        self.sim.autopilot=value
        self.sim.last_dwa_t=-100
        self.set_message('Controle autonomo com A* e DWA.' if value
                         else 'Controle manual: use W A S D ou setas.')

    def map_controls(self,surf):
        # Transparent control band over the map bottom
        rounded(surf,(43,724,1047,40),(12,29,45),9,(49,81,100))
        self.create_button(surf,'CLIQUE: DESTINO',(53,729,152,30),lambda:self.set_edit('goal'),self.map_edit=='goal',fontsize=12)
        self.create_button(surf,'+ OBSTACULO',(212,729,140,30),lambda:self.set_edit('add'),self.map_edit=='add',fontsize=12)
        self.create_button(surf,'REMOVER',(359,729,100,30),lambda:self.set_edit('remove'),self.map_edit=='remove',fontsize=12)
        pygame.draw.line(surf,BORDER,(471,733),(471,752),1)
        for label,name,x,width in [('LiDAR','show_lidar',482,87),('ROTA A*','show_path',576,100),
                                   ('BUSCA','show_search',684,93),('ROTA Q','show_q',785,94),
                                   ('MAPA Q','show_heat',887,91),('AGVs','dynamic_enabled',985,92)]:
            self.create_button(surf,label,(x,729,width,30),lambda n=name:self.toggle(n),getattr(self.sim,name),fontsize=11)

    def set_edit(self,mode):
        self.map_edit=mode
        names={'goal':'Clique no mapa para definir o destino.',
               'add':'Clique no mapa para inserir uma caixa.',
               'remove':'Clique em uma caixa adicionada para remove-la.'}
        self.set_message(names[mode])

    def bottom_bar(self,surf):
        s=self.sim
        rounded(surf,(28,790,1080,103),CARD,13,BORDER)
        metrics=[
            ('MISSAO','CONCLUIDA' if s.reached else ('SEM ROTA' if not s.path else 'NAVEGANDO'),LIME if s.reached else CYAN),
            ('DISTANCIA','{:.0f} unid.'.format(s.travelled),WHITE),
            ('TEMPO','{:.1f} s'.format(s.time),WHITE),
            ('PARADAS SEGURAS',str(s.collisions_avoided),ORANGE),
        ]
        widths=(232,222,193,231)
        x=43
        for i,((label,val,col),w) in enumerate(zip(metrics,widths)):
            text(surf,label,x+10,807,12,MUTED,True)
            text(surf,val,x+10,826,23,col,True)
            if i<3:
                pygame.draw.line(surf,BORDER,(x+w,806),(x+w,856),1)
            x+=w+13
        pygame.draw.line(surf,BORDER,(42,866),(1092,866),1)
        text(surf,'EDITAR MAPA: clique esquerdo  •  remover caixa: direito  •  W/A/S/D: dirigir  •  ESPACO: pausar  •  T: treinar Q-learning',
             51,874,12,MUTED)

    def draw_help(self,surf):
        overlay=pygame.Surface((VIRTUAL_W,VIRTUAL_H),pygame.SRCALPHA)
        overlay.fill((2,7,13,197))
        surf.blit(overlay,(0,0))
        rect=pygame.Rect(270,134,996,640)
        rounded(surf,rect,(16,31,48),17,(83,147,170),2)
        text(surf,'GUIA DO LABORATORIO',313,162,29,WHITE,True)
        text(surf,'Como explicar IA em robotica usando este simulador',314,208,16,MUTED)
        topics=[
            ('01  PERCEPCAO', 'O LiDAR virtual emite raios e calcula distancias a objetos. Os pontos cianos sao retornos simulados.'),
            ('02  MAPA E PLANEJAMENTO A*', 'O A* busca uma rota no mapa conhecido. Ative BUSCA para mostrar as celulas exploradas.'),
            ('03  CONTROLE LOCAL DWA', 'O controlador avalia dezenas de movimentos previstos. A trajetoria verde e a escolha de melhor pontuacao.'),
            ('04  OBSTACULOS DINAMICOS', 'Os AGVs laranja cruzam as rotas. O robo recalcula o caminho e pode parar por seguranca.'),
            ('05  APRENDIZADO Q-LEARNING', 'Treinar IA executa episodios de tentativa e erro na grade. A rota roxa mostra a politica aprendida, se viavel.'),
            ('06  AUTONOMIA VERSUS CONTROLE MANUAL', 'Troque AUTOMOMO/MANUAL. Na direcao manual use as teclas W A S D ou setas.'),
        ]
        y=249
        for title,description in topics:
            pygame.draw.circle(surf,CYAN,(322,y+8),4)
            text(surf,title,341,y-2,16,CYAN,True)
            for j,line in enumerate(wrap_text(description,91,2)):
                text(surf,line,341,y+22+16*j,14,WHITE)
            y+=73
        rounded(surf,(316,707,899,40),(24,50,69),9,(61,105,127))
        text(surf,'ATENCAO: A* e busca classica; Q-learning aprende uma politica. Nao ha sensores reais nem rede neural.',
             329,719,13,WHITE)
        text(surf,'Clique em qualquer lugar para fechar',770,768,13,MUTED,anchor='midbottom')

    def draw(self):
        self.buttons=[]
        surf=self.canvas
        surf.fill(BG)
        self.header(surf)
        # Maps have original resolution; scale exactly to 1080 x 670.
        world=self.map_surface()
        surf.blit(pygame.transform.smoothscale(world,MAP.size),MAP.topleft)
        pygame.draw.rect(surf,(62,108,134),MAP,2,border_radius=4)
        self.map_controls(surf)
        self.sidebar(surf)
        self.bottom_bar(surf)
        if self.sim.is_paused and not self.help_open:
            rounded(surf,(390,115,345,42),(20,48,68),9,CYAN,1)
            text(surf,'SIMULACAO PAUSADA',562,136,17,WHITE,True,'center')
        if self.help_open:
            self.draw_help(surf)
        self.present()

    def present(self):
        w,h=self.screen.get_size()
        scale=min(w/VIRTUAL_W,h/VIRTUAL_H)
        rw,rh=round(VIRTUAL_W*scale),round(VIRTUAL_H*scale)
        self.screen.fill((4,10,19))
        if (rw,rh)==(VIRTUAL_W,VIRTUAL_H):
            self.screen.blit(self.canvas,(round((w-rw)/2),round((h-rh)/2)))
        else:
            self.screen.blit(pygame.transform.smoothscale(self.canvas,(rw,rh)),
                             (round((w-rw)/2),round((h-rh)/2)))
        pygame.display.flip()

    def capture(self):
        folder=Path(__file__).resolve().parent/'capturas'
        folder.mkdir(exist_ok=True)
        path=folder/('robo_lab_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.png')
        pygame.image.save(self.canvas,str(path))
        self.set_message('Captura salva em capturas/{}'.format(path.name))

    def loop(self):
        while self.running:
            dt=self.clock.tick(FPS)/1000
            self.ui_time+=dt
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    self.running=False
                elif event.type==pygame.VIDEORESIZE:
                    self.screen=pygame.display.set_mode((max(900,event.w),max(600,event.h)),pygame.RESIZABLE)
                elif event.type==pygame.MOUSEBUTTONDOWN:
                    self.mouse_click(self.coordinates(event.pos),event.button)
                elif event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_ESCAPE:
                        self.help_open=False if self.help_open else True
                    elif event.key==pygame.K_SPACE:
                        self.pause()
                    elif event.key==pygame.K_r:
                        self.change_scene(self.sim.scenario)
                    elif event.key==pygame.K_q or event.key==pygame.K_t:
                        self.sim.toggle_train()
                    elif event.key==pygame.K_l:
                        self.toggle('show_lidar')
                    elif event.key==pygame.K_p:
                        self.toggle('show_path')
                    elif event.key==pygame.K_h:
                        self.toggle('show_search')
                    elif event.key==pygame.K_F12:
                        self.capture()
                    elif event.key==pygame.K_1:
                        self.change_scene('armazem')
                    elif event.key==pygame.K_2:
                        self.change_scene('resgate')
                    elif event.key==pygame.K_3:
                        self.change_scene('cidade')
                    elif event.key==pygame.K_m:
                        self.set_auto(not self.sim.autopilot)
            keys=pygame.key.get_pressed()
            forward=int(bool(keys[pygame.K_w] or keys[pygame.K_UP]))-int(bool(keys[pygame.K_s] or keys[pygame.K_DOWN]))
            angular=int(bool(keys[pygame.K_d] or keys[pygame.K_RIGHT]))-int(bool(keys[pygame.K_a] or keys[pygame.K_LEFT]))
            if not self.help_open:
                self.sim.step(dt,(forward,angular))
            self.draw()
        pygame.quit()


def main():
    try:
        RoboLab().loop()
    except Exception:
        import traceback
        trace=traceback.format_exc()
        log_path=Path(__file__).resolve().parent/'robotica_erros.log'
        log_path.write_text(trace,encoding='utf-8')
        print(trace,file=sys.stderr)
        print('Detalhes em:',log_path,file=sys.stderr)
        raise

if __name__=='__main__':
    main()
