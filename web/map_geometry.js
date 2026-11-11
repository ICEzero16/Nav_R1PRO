'use strict';
const MapGeometry={
 worldToScreen(x,y,view,w,h){return [w/2+(x-view.x)*view.scale,h/2-(y-view.y)*view.scale];},
 imageTransform(meta,view,w,h){const [x,y,a]=meta.origin,s=meta.resolution*view.scale,c=Math.cos(a),t=Math.sin(a),p=this.worldToScreen(x,y,view,w,h);return [c*s,-t*s,t*s,c*s,p[0]-t*s*meta.height,p[1]-c*s*meta.height];},
 bounds(meta){const [x,y,a]=meta.origin,c=Math.cos(a),s=Math.sin(a);return [[0,0],[meta.width,0],[0,meta.height],[meta.width,meta.height]].map(([u,v])=>[x+meta.resolution*(c*u-s*v),y+meta.resolution*(s*u+c*v)]);}
};
if(typeof module!=='undefined')module.exports=MapGeometry;
