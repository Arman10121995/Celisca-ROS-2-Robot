"""Pure graph rendering helpers and opt-in, owned read-only ROS discovery."""
from datetime import datetime, timezone
import os
import re
import time
import uuid
import xml.etree.ElementTree as ET

COLORS = {'Interface':'#72b7ff', 'Configuration':'#b59bff', 'Orchestration':'#c0a1ff',
          'Assets':'#d7ae75', 'Simulation':'#f49879', 'ROS contract':'#65cdd0',
          'Estimation':'#73d5ad', 'Planning':'#8fc978', 'Control':'#ffc974',
          'Manipulation':'#ed9bd6', 'Aerial':'#80c4fa', 'Fleet':'#b8b0ff',
          'Marine':'#69d3d4', 'Evidence':'#c1ccdf', 'Networking':'#92aafb',
          'ROS node':'#77b7ff', 'ROS topic':'#79d4aa'}
WIDTH, HEIGHT = 230, 92


def edge_points(source, target):
    """Rectilinear routes connect rectangle boundaries, not card centers."""
    x, y = source; u, v = target
    if abs(u-x) >= abs(v-y):
        a = (x + (WIDTH if u>x else 0), y+HEIGHT/2)
        b = (u + (0 if u>x else WIDTH), v+HEIGHT/2)
        mid=(a[0]+b[0])/2
        return [a, (mid,a[1]), (mid,b[1]), b]
    a=(x+WIDTH/2,y+(HEIGHT if v>y else 0))
    b=(u+WIDTH/2,v+(0 if v>y else HEIGHT))
    mid=(a[1]+b[1])/2
    return [a,(a[0],mid),(b[0],mid),b]


def graph_extent(view):
    points=list(view['positions'].values()) or [(0,0)]
    return max(x for x,y in points)+WIDTH+70, max(y for x,y in points)+HEIGHT+60


def svg_diagram(view, components):
    """Self-contained, escaped vector export; no browser libraries or network."""
    width,height=graph_extent(view)
    svg=ET.Element('svg',xmlns='http://www.w3.org/2000/svg',viewBox=f'0 0 {width} {height}',role='img')
    ET.SubElement(svg,'title').text=view['title']
    ET.SubElement(svg,'desc').text=view.get('subtitle','')+'; designed diagram, not live telemetry.'
    ET.SubElement(svg,'rect',width='100%',height='100%',fill='#101824')
    defs=ET.SubElement(svg,'defs');marker=ET.SubElement(defs,'marker',id='arrow',viewBox='0 0 10 10',refX='9',refY='5',markerWidth='7',markerHeight='7',orient='auto-start-reverse')
    ET.SubElement(marker,'path',d='M 0 0 L 10 5 L 0 10 z',fill='#7c93ad')
    ET.SubElement(svg,'text',x='35',y='32',fill='#edf3fc',**{'font-family':'sans-serif','font-size':'20'}).text=view['title']
    for edge in view['edges']:
        points=edge_points(view['positions'][edge['source']],view['positions'][edge['target']])
        ET.SubElement(svg,'polyline',points=' '.join(f'{x},{y}' for x,y in points),fill='none',stroke='#7c93ad',**{'stroke-width':'1.7','marker-end':'url(#arrow)'})
        a,b=points[1:3]
        ET.SubElement(svg,'text',x=str((a[0]+b[0])/2+5),y=str((a[1]+b[1])/2-7),fill='#b8cadd',**{'font-family':'sans-serif','font-size':'10'}).text=edge['label']
    import textwrap
    for key,(x,y) in view['positions'].items():
        c=components[key];color=COLORS.get(c['layer'],'#90b7f0')
        group=ET.SubElement(svg,'g');ET.SubElement(group,'title').text=c['summary']
        ET.SubElement(group,'rect',x=str(x),y=str(y),width=str(WIDTH),height=str(HEIGHT),rx='12',fill='#1d2a3d',stroke=color)
        for offset,text,size,fill in [(19,c['layer'].upper(),9,color),(40,c['title'],13,'#f1f5fc')]:
            ET.SubElement(group,'text',x=str(x+12),y=str(y+offset),fill=fill,**{'font-family':'sans-serif','font-size':str(size)}).text=text
        for i,line in enumerate(textwrap.wrap(c['summary'],35)[:2]):
            ET.SubElement(group,'text',x=str(x+12),y=str(y+59+i*13),fill='#bccbdf',**{'font-family':'sans-serif','font-size':'10'}).text=line
    return ET.tostring(svg,encoding='unicode')


def resolve_topic(namespace, topic):
    namespace='/' + namespace.strip('/')
    value=topic if topic.startswith('/') else namespace.rstrip('/')+'/'+topic
    if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*',value):
        raise ValueError('Use ROS example names such as /alpha and cmd_vel; private ~ names are not modeled here.')
    return value


def network_lesson(values):
    """Illustrative compatibility gates only; never alters or tests host networking."""
    try:
        publisher_domain,subscriber_domain=int(values['publisher_domain']),int(values['subscriber_domain'])
        if not (0<=publisher_domain<=232 and 0<=subscriber_domain<=232):
            raise ValueError('Choose domains 0–232 for this lesson; platform/port constraints can narrow the usable range.')
        published=resolve_topic(values['publisher_ns'],values['publisher_topic'])
        subscribed=resolve_topic(values['subscriber_ns'],values['subscriber_topic'])
        ranks={'best_effort':0,'reliable':1,'volatile':0,'transient_local':1}
        checks=[('DDS domain',publisher_domain==subscriber_domain,'Domain IDs must match.'),
                ('Resolved topic',published==subscribed,published+' → '+subscribed),
                ('Message type',values['publisher_type']==values['subscriber_type'],'Publisher and subscription types must match.'),
                ('Reliability',ranks[values['publisher_reliability']]>=ranks[values['subscriber_reliability']],'Offered reliability must satisfy requested reliability.'),
                ('Durability',ranks[values['publisher_durability']]>=ranks[values['subscriber_durability']],'A volatile publisher cannot satisfy a transient-local request.')]
        return checks
    except (KeyError,ValueError) as error:
        return [('Example input',False,str(error))]


def discover_ros(cancel, duration=2., max_nodes=80, max_topics=160):
    """Own context/executor; introspection only, no robot subscriptions or clients.

    The Node constructor creates its standard parameter-event publisher; there
    are no application command publishers. Nothing here calls a robot service.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context=Context();node=None;executor=None
    errors=[];rows=[];topics={};endpoint_qos={};truncated=False
    try:
        context.init(args=[])
        node=rclpy.create_node('robot_lab_architecture_'+uuid.uuid4().hex[:10],context=context,
                               enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline and not cancel.is_set():
            executor.spin_once(timeout_sec=.05)
        names=sorted(set(node.get_node_names_and_namespaces()))
        names=[(name,ns) for name,ns in names if name!=node.get_name()]
        truncated=len(names)>max_nodes;query_deadline=time.monotonic()+3.
        for name,ns in names[:max_nodes]:
            if cancel.is_set() or time.monotonic()>query_deadline:
                truncated=True;break
            full=ns.rstrip('/')+'/'+name
            entry=dict(name=full,namespace=ns,publishers=[],subscribers=[],services=[])
            try:
                for role,method in [('publishers',node.get_publisher_names_and_types_by_node),
                                    ('subscribers',node.get_subscriber_names_and_types_by_node),
                                    ('services',node.get_service_names_and_types_by_node)]:
                    for topic,types in method(name,ns):
                        if role!='services' and topic not in topics and len(topics)>=max_topics:
                            truncated=True;continue
                        entry[role].append(dict(name=topic,types=types))
                        if role!='services':topics.setdefault(topic,set()).update(types)
            except Exception as error:  # A remote node can disappear during discovery.
                errors.append(full+': '+str(error))
            rows.append(entry)
        for topic in sorted(topics)[:40]:
            if cancel.is_set() or time.monotonic()>query_deadline:break
            endpoints={}
            try:
                for role,method in [('offered',node.get_publishers_info_by_topic),('requested',node.get_subscriptions_info_by_topic)]:
                    endpoints[role]=[]
                    for info in method(topic)[:40]:
                        qos=info.qos_profile
                        endpoints[role].append(dict(node=info.node_namespace.rstrip('/')+'/'+info.node_name,
                            type=info.topic_type,reliability=getattr(qos.reliability,'name',str(qos.reliability)),
                            durability=getattr(qos.durability,'name',str(qos.durability)),
                            history=getattr(qos.history,'name',str(qos.history)),depth=qos.depth))
                endpoint_qos[topic]=endpoints
            except Exception as error:errors.append(topic+': '+str(error))
        return dict(observed_at=datetime.now(timezone.utc).isoformat(),domain=os.environ.get('ROS_DOMAIN_ID','0'),
                    rmw=rclpy.get_rmw_implementation_identifier(),nodes=rows,
                    topics={key:sorted(types) for key,types in topics.items()},endpoint_qos=endpoint_qos,
                    qos_scope='Advertised QoS for up to 40 topics and 40 endpoints per role; omitted entries are unqueried, not incompatible.',
                    truncated=truncated,errors=errors,
                    scope='Discovery snapshot; endpoints are not proof of QoS compatibility, traffic, readiness or a working mission.')
    finally:
        if executor is not None:
            if node is not None:executor.remove_node(node)
            executor.shutdown(timeout_sec=.5)
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()


def observed_view(snapshot, query='', hide_infrastructure=True):
    """Bounded bipartite node/topic graph, including direction and actual types."""
    components={};edges=[];positions={}
    nodes=snapshot.get('nodes',[])
    query=query.casefold().strip()
    hidden={'/rosout','/parameter_events','/tf','/tf_static','/clock'} if hide_infrastructure else set()
    selected=[row for row in nodes if not query or query in row['name'].casefold() or
              any(query in item['name'].casefold() for role in ('publishers','subscribers') for item in row[role])][:40]
    topics=sorted({item['name'] for row in selected for role in ('publishers','subscribers')
                   for item in row[role] if item['name'] not in hidden})[:60]
    for i,row in enumerate(selected):
        key='node:'+row['name'];positions[key]=(60,i*125+90)
        components[key]=dict(id=key,title=row['name'],layer='ROS node',summary='Observed node; click for advertised publishers, subscribers and services.',
            detail='Discovery snapshot only.\n\n'+ '\n\n'.join(role.upper()+'\n'+('\n'.join(item['name']+' : '+', '.join(item['types']) for item in row[role]) or '(none discovered)') for role in ('publishers','subscribers','services')),
            sources=[],guide='docs/architecture/networking.md',ports='',observed=True)
        for role in ('publishers','subscribers'):
            for item in row[role]:
                if item['name'] in topics:
                    topic_key='topic:'+item['name']
                    edges.append(dict(source=key if role=='publishers' else topic_key,
                                      target=topic_key if role=='publishers' else key,label='publishes' if role=='publishers' else 'subscribes'))
    for i,topic in enumerate(topics):
        key='topic:'+topic;positions[key]=(490,i*125+90)
        types=snapshot['topics'].get(topic,[])
        qos=snapshot.get('endpoint_qos',{}).get(topic)
        detail='Advertised topic types:\n'+ '\n'.join(types)
        if qos is not None:
            for role in ('offered','requested'):
                detail+='\n\n'+role.upper()+' ENDPOINT QoS\n'+('\n'.join(
                    row['node']+'\n  '+row['reliability']+' / '+row['durability']+' / '+row['history']+' depth '+str(row['depth'])
                    for row in qos[role]) or '(none discovered)')
        else:detail+='\n\nEndpoint QoS was not queried for this topic in the bounded snapshot.'
        components[key]=dict(id=key,title=topic,layer='ROS topic',summary=', '.join(types),
            detail=detail+'\n\nEdges and QoS are advertised metadata, not measured data delivery. For rate/TF inspect Live Monitor and the relevant node configuration.',
            sources=[],guide='docs/architecture/networking.md',ports='',observed=True)
    if not positions:
        positions={'empty':(80,100)}
        components['empty']=dict(id='empty',title='No matching endpoints',layer='ROS node',summary='Start a compatible workflow in this domain, or clear the filter.',detail=snapshot.get('scope',''),sources=[],ports='',guide='docs/architecture/networking.md')
    return dict(id='observed',title='Observed ROS graph',subtitle='Read-only discovery snapshot; filtered display capped at 40 nodes / 60 topics',positions=positions,edges=edges[:400],lesson=snapshot.get('scope','')),components
