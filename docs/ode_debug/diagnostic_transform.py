import os
from pathlib import Path

def transform(doc):
    mode=os.environ.get('ODE_CASE','baseline')
    tool={'plate_frame'}
    joints=doc.getElementsByTagName('joint')
    for _ in range(30):
        for j in joints:
            pa=j.getElementsByTagName('parent');ch=j.getElementsByTagName('child')
            if pa and ch and pa[0].getAttribute('link') in tool:tool.add(ch[0].getAttribute('link'))
    for link in doc.getElementsByTagName('link'):
        if (mode=='no_tool_collision' and link.getAttribute('name') in tool) or mode in ['no_collision','static_no_collision'] or (mode=='static_no_base_collision' and link.getAttribute('name')=='fr10_base_link'):
            for c in list(link.getElementsByTagName('collision')):link.removeChild(c)
    if mode=='vehicle_no_wheel_collision':
        for link in doc.getElementsByTagName('link'):
            if link.getAttribute('name').startswith('Wheel'):
                for c in list(link.getElementsByTagName('collision')):link.removeChild(c)
    if mode=='vehicle_no_arm_collision':
        names=tool|{'fr10_base_link','shoulder_link','upperarm_link','forearm_link','wrist1_link','wrist2_link','wrist3_link'}
        for link in doc.getElementsByTagName('link'):
            if link.getAttribute('name') in names:
                for c in list(link.getElementsByTagName('collision')):link.removeChild(c)
    if mode.startswith('base_clearance'):
        for link in doc.getElementsByTagName('link'):
            if link.getAttribute('name') == 'fr10_base_link':
                for col in link.getElementsByTagName('collision'):
                    geometry=col.getElementsByTagName('geometry')[0]
                    for child in list(geometry.childNodes):geometry.removeChild(child)
                    cyl=doc.createElement('cylinder');cyl.setAttribute('radius','0.095');cyl.setAttribute('length','0.0838');geometry.appendChild(cyl)
                    origin=col.getElementsByTagName('origin')[0]
                    origin.setAttribute('xyz','0 0 0.0429');origin.setAttribute('rpy','0 0 0')
    Path(os.environ['ODE_OUT']+'/model.urdf').write_text(doc.toxml())
