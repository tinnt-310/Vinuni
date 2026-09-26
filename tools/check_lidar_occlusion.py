from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh,json,sys
root=ET.parse(sys.argv[1]).getroot();links={x.get('name'):x for x in root.findall('link')};joints={x.find('child').get('link'):x for x in root.findall('joint')}
def origin(e):
 o=e.find('origin');m=np.eye(4)
 if o is not None:
  m=trimesh.transformations.euler_matrix(*map(float,o.get('rpy','0 0 0').split()));m[:3,3]=list(map(float,o.get('xyz','0 0 0').split()))
 return m
def tf(name,q):
 if name=='base_footprint':return np.eye(4)
 j=joints[name];m=origin(j)
 if j.get('type') in ['continuous','revolute']:
  m=m@trimesh.transformations.rotation_matrix(q.get(j.get('name'),0),list(map(float,j.find('axis').get('xyz').split())))
 return tf(j.find('parent').get('link'),q)@m
meshes={}
for name,l in links.items():
 if name in ['lidar_link','laser_frame','lidar_mount_frame']:continue
 for v in l.findall('visual'):
  g=v.find('geometry');e=g[0]
  if e.tag=='mesh':
   uri=e.get('filename');pkg,rel=uri.split('package://')[1].split('/',1)
   path=Path(__file__).resolve().parents[1]/'install'/pkg/'share'/pkg/rel
   if not path.exists():path=Path('/opt/ros/humble/share')/pkg/rel
   m=trimesh.load(str(path),force='mesh');m.apply_scale(list(map(float,e.get('scale','1 1 1').split())))
  elif e.tag=='box':m=trimesh.creation.box(list(map(float,e.get('size').split())))
  elif e.tag=='cylinder':m=trimesh.creation.cylinder(float(e.get('radius')),float(e.get('length')))
  else:continue
  m.apply_transform(origin(v));meshes.setdefault(name,[]).append(m)
angles=np.linspace(-np.pi,np.pi,720,endpoint=False);dirs=np.array([np.cos(angles),np.sin(angles)]).T
results=[]
for label,values in [('home',[0,-np.pi/2,0,-np.pi/2,0,0]),('reach',[0,-1.0,.5,-1.0,0,0]),('side',[np.pi/2,-1.0,.5,-1.0,0,0])]:
 q={f'j{i+1}':v for i,v in enumerate(values)};blocked=np.zeros(720,bool);hitlinks=[]
 for name,parts in meshes.items():
  mat=tf(name,q);hit=False
  for part in parts:
   m=part.copy();m.apply_transform(mat)
   seg=trimesh.intersections.mesh_plane(m,[0,0,1],[.275,0,1.0415])
   if len(seg)==0:continue
   a=seg[:,0,:2]-[.275,0];b=seg[:,1,:2]-seg[:,0,:2]
   den=dirs[:,None,0]*b[None,:,1]-dirs[:,None,1]*b[None,:,0]
   den=np.where(abs(den)<1e-12,np.nan,den)
   t=(a[:,0]*b[:,1]-a[:,1]*b[:,0])[None,:]/den
   u=(a[None,:,0]*dirs[:,None,1]-a[None,:,1]*dirs[:,None,0])/den
   hits=np.any((t>0)&(u>=0)&(u<=1),axis=1)
   if hits.any():hit=True;blocked|=hits
  if hit:hitlinks.append(name)
 front=abs(angles)<np.pi/2
 result={'pose':label,'joints':values,'blocked_degrees':float(blocked.sum()/2),'front_180_blocked_degrees':float((blocked&front).sum()/2),'links':hitlinks}
 print(result,flush=True);results.append(result)
Path('/tmp/vehicle-lidar-occlusion.json').write_text(json.dumps(results,indent=2))
