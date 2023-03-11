import{EventDispatcher,MOUSE,Quaternion,Spherical,TOUCH,Vector2,Vector3}from"/lib/three.js";const _changeEvent={type:"change"},_startEvent={type:"start"},_endEvent={type:"end"};class OrbitControls extends EventDispatcher{constructor(e,t){super(),this.object=e,this.domElement=t,this.domElement.style.touchAction="none",this.enabled=!0,this.target=new Vector3,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:MOUSE.ROTATE,MIDDLE:MOUSE.DOLLY,RIGHT:MOUSE.PAN},this.touches={ONE:TOUCH.ROTATE,TWO:TOUCH.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this.getPolarAngle=function(){return r.phi},this.getAzimuthalAngle=function(){return r.theta},this.getDistance=function(){return this.object.position.distanceTo(this.target)},this.listenToKeyEvents=function(e){e.addEventListener("keydown",z),this._domElementKeyEvents=e},this.stopListenToKeyEvents=function(){this._domElementKeyEvents.removeEventListener("keydown",z),this._domElementKeyEvents=null},this.saveState=function(){n.target0.copy(n.target),n.position0.copy(n.object.position),n.zoom0=n.object.zoom},this.reset=function(){n.target.copy(n.target0),n.object.position.copy(n.position0),n.object.zoom=n.zoom0,n.object.updateProjectionMatrix(),n.dispatchEvent(_changeEvent),n.update(),a=o.NONE},this.update=function(){const t=new Vector3,p=(new Quaternion).setFromUnitVectors(e.up,new Vector3(0,1,0)),h=p.clone().invert(),u=new Vector3,d=new Quaternion,E=2*Math.PI;return function(){const e=n.object.position;t.copy(e).sub(n.target),t.applyQuaternion(p),r.setFromVector3(t),n.autoRotate&&a===o.NONE&&P(2*Math.PI/60/60*n.autoRotateSpeed),n.enableDamping?(r.theta+=c.theta*n.dampingFactor,r.phi+=c.phi*n.dampingFactor):(r.theta+=c.theta,r.phi+=c.phi);let b=n.minAzimuthAngle,g=n.maxAzimuthAngle;return isFinite(b)&&isFinite(g)&&(b<-Math.PI?b+=E:b>Math.PI&&(b-=E),g<-Math.PI?g+=E:g>Math.PI&&(g-=E),r.theta=b<=g?Math.max(b,Math.min(g,r.theta)):r.theta>(b+g)/2?Math.max(b,r.theta):Math.min(g,r.theta)),r.phi=Math.max(n.minPolarAngle,Math.min(n.maxPolarAngle,r.phi)),r.makeSafe(),r.radius*=s,r.radius=Math.max(n.minDistance,Math.min(n.maxDistance,r.radius)),!0===n.enableDamping?n.target.addScaledVector(l,n.dampingFactor):n.target.add(l),t.setFromSpherical(r),t.applyQuaternion(h),e.copy(n.target).add(t),n.object.lookAt(n.target),!0===n.enableDamping?(c.theta*=1-n.dampingFactor,c.phi*=1-n.dampingFactor,l.multiplyScalar(1-n.dampingFactor)):(c.set(0,0,0),l.set(0,0,0)),s=1,!!(m||u.distanceToSquared(n.object.position)>i||8*(1-d.dot(n.object.quaternion))>i)&&(n.dispatchEvent(_changeEvent),u.copy(n.object.position),d.copy(n.object.quaternion),m=!1,!0)}}(),this.dispose=function(){n.domElement.removeEventListener("contextmenu",F),n.domElement.removeEventListener("pointerdown",I),n.domElement.removeEventListener("pointercancel",V),n.domElement.removeEventListener("wheel",K),n.domElement.removeEventListener("pointermove",D),n.domElement.removeEventListener("pointerup",U),null!==n._domElementKeyEvents&&(n._domElementKeyEvents.removeEventListener("keydown",z),n._domElementKeyEvents=null)};const n=this,o={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6};let a=o.NONE;const i=1e-6,r=new Spherical,c=new Spherical;let s=1;const l=new Vector3;let m=!1;const p=new Vector2,h=new Vector2,u=new Vector2,d=new Vector2,E=new Vector2,b=new Vector2,g=new Vector2,f=new Vector2,O=new Vector2,y=[],T={};function v(){return Math.pow(.95,n.zoomSpeed)}function P(e){c.theta-=e}function A(e){c.phi-=e}const L=function(){const e=new Vector3;return function(t,n){e.setFromMatrixColumn(n,0),e.multiplyScalar(-t),l.add(e)}}(),M=function(){const e=new Vector3;return function(t,o){!0===n.screenSpacePanning?e.setFromMatrixColumn(o,1):(e.setFromMatrixColumn(o,0),e.crossVectors(n.object.up,e)),e.multiplyScalar(t),l.add(e)}}(),N=function(){const e=new Vector3;return function(t,o){const a=n.domElement;if(n.object.isPerspectiveCamera){const i=n.object.position;e.copy(i).sub(n.target);let r=e.length();r*=Math.tan(n.object.fov/2*Math.PI/180),L(2*t*r/a.clientHeight,n.object.matrix),M(2*o*r/a.clientHeight,n.object.matrix)}else n.object.isOrthographicCamera?(L(t*(n.object.right-n.object.left)/n.object.zoom/a.clientWidth,n.object.matrix),M(o*(n.object.top-n.object.bottom)/n.object.zoom/a.clientHeight,n.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),n.enablePan=!1)}}();function w(e){n.object.isPerspectiveCamera?s/=e:n.object.isOrthographicCamera?(n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom*e)),n.object.updateProjectionMatrix(),m=!0):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function S(e){n.object.isPerspectiveCamera?s*=e:n.object.isOrthographicCamera?(n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom/e)),n.object.updateProjectionMatrix(),m=!0):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function j(e){p.set(e.clientX,e.clientY)}function C(e){d.set(e.clientX,e.clientY)}function R(){if(1===y.length)p.set(y[0].pageX,y[0].pageY);else{const e=.5*(y[0].pageX+y[1].pageX),t=.5*(y[0].pageY+y[1].pageY);p.set(e,t)}}function k(){if(1===y.length)d.set(y[0].pageX,y[0].pageY);else{const e=.5*(y[0].pageX+y[1].pageX),t=.5*(y[0].pageY+y[1].pageY);d.set(e,t)}}function _(){const e=y[0].pageX-y[1].pageX,t=y[0].pageY-y[1].pageY,n=Math.sqrt(e*e+t*t);g.set(0,n)}function x(e){if(1==y.length)h.set(e.pageX,e.pageY);else{const t=B(e),n=.5*(e.pageX+t.x),o=.5*(e.pageY+t.y);h.set(n,o)}u.subVectors(h,p).multiplyScalar(n.rotateSpeed);const t=n.domElement;P(2*Math.PI*u.x/t.clientHeight),A(2*Math.PI*u.y/t.clientHeight),p.copy(h)}function H(e){if(1===y.length)E.set(e.pageX,e.pageY);else{const t=B(e),n=.5*(e.pageX+t.x),o=.5*(e.pageY+t.y);E.set(n,o)}b.subVectors(E,d).multiplyScalar(n.panSpeed),N(b.x,b.y),d.copy(E)}function Y(e){const t=B(e),o=e.pageX-t.x,a=e.pageY-t.y,i=Math.sqrt(o*o+a*a);f.set(0,i),O.set(0,Math.pow(f.y/g.y,n.zoomSpeed)),w(O.y),g.copy(f)}function I(e){!1!==n.enabled&&(0===y.length&&(n.domElement.setPointerCapture(e.pointerId),n.domElement.addEventListener("pointermove",D),n.domElement.addEventListener("pointerup",U)),function(e){y.push(e)}(e),"touch"===e.pointerType?function(e){switch(Z(e),y.length){case 1:switch(n.touches.ONE){case TOUCH.ROTATE:if(!1===n.enableRotate)return;R(),a=o.TOUCH_ROTATE;break;case TOUCH.PAN:if(!1===n.enablePan)return;k(),a=o.TOUCH_PAN;break;default:a=o.NONE}break;case 2:switch(n.touches.TWO){case TOUCH.DOLLY_PAN:if(!1===n.enableZoom&&!1===n.enablePan)return;n.enableZoom&&_(),n.enablePan&&k(),a=o.TOUCH_DOLLY_PAN;break;case TOUCH.DOLLY_ROTATE:if(!1===n.enableZoom&&!1===n.enableRotate)return;n.enableZoom&&_(),n.enableRotate&&R(),a=o.TOUCH_DOLLY_ROTATE;break;default:a=o.NONE}break;default:a=o.NONE}a!==o.NONE&&n.dispatchEvent(_startEvent)}(e):function(e){let t;switch(e.button){case 0:t=n.mouseButtons.LEFT;break;case 1:t=n.mouseButtons.MIDDLE;break;case 2:t=n.mouseButtons.RIGHT;break;default:t=-1}switch(t){case MOUSE.DOLLY:if(!1===n.enableZoom)return;!function(e){g.set(e.clientX,e.clientY)}(e),a=o.DOLLY;break;case MOUSE.ROTATE:if(e.ctrlKey||e.metaKey||e.shiftKey){if(!1===n.enablePan)return;C(e),a=o.PAN}else{if(!1===n.enableRotate)return;j(e),a=o.ROTATE}break;case MOUSE.PAN:if(e.ctrlKey||e.metaKey||e.shiftKey){if(!1===n.enableRotate)return;j(e),a=o.ROTATE}else{if(!1===n.enablePan)return;C(e),a=o.PAN}break;default:a=o.NONE}a!==o.NONE&&n.dispatchEvent(_startEvent)}(e))}function D(e){!1!==n.enabled&&("touch"===e.pointerType?function(e){switch(Z(e),a){case o.TOUCH_ROTATE:if(!1===n.enableRotate)return;x(e),n.update();break;case o.TOUCH_PAN:if(!1===n.enablePan)return;H(e),n.update();break;case o.TOUCH_DOLLY_PAN:if(!1===n.enableZoom&&!1===n.enablePan)return;!function(e){n.enableZoom&&Y(e),n.enablePan&&H(e)}(e),n.update();break;case o.TOUCH_DOLLY_ROTATE:if(!1===n.enableZoom&&!1===n.enableRotate)return;!function(e){n.enableZoom&&Y(e),n.enableRotate&&x(e)}(e),n.update();break;default:a=o.NONE}}(e):function(e){switch(a){case o.ROTATE:if(!1===n.enableRotate)return;!function(e){h.set(e.clientX,e.clientY),u.subVectors(h,p).multiplyScalar(n.rotateSpeed);const t=n.domElement;P(2*Math.PI*u.x/t.clientHeight),A(2*Math.PI*u.y/t.clientHeight),p.copy(h),n.update()}(e);break;case o.DOLLY:if(!1===n.enableZoom)return;!function(e){f.set(e.clientX,e.clientY),O.subVectors(f,g),O.y>0?w(v()):O.y<0&&S(v()),g.copy(f),n.update()}(e);break;case o.PAN:if(!1===n.enablePan)return;!function(e){E.set(e.clientX,e.clientY),b.subVectors(E,d).multiplyScalar(n.panSpeed),N(b.x,b.y),d.copy(E),n.update()}(e)}}(e))}function U(e){X(e),0===y.length&&(n.domElement.releasePointerCapture(e.pointerId),n.domElement.removeEventListener("pointermove",D),n.domElement.removeEventListener("pointerup",U)),n.dispatchEvent(_endEvent),a=o.NONE}function V(e){X(e)}function K(e){!1!==n.enabled&&!1!==n.enableZoom&&a===o.NONE&&(e.preventDefault(),n.dispatchEvent(_startEvent),function(e){e.deltaY<0?S(v()):e.deltaY>0&&w(v()),n.update()}(e),n.dispatchEvent(_endEvent))}function z(e){!1!==n.enabled&&!1!==n.enablePan&&function(e){let t=!1;switch(e.code){case n.keys.UP:e.ctrlKey||e.metaKey||e.shiftKey?A(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):N(0,n.keyPanSpeed),t=!0;break;case n.keys.BOTTOM:e.ctrlKey||e.metaKey||e.shiftKey?A(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):N(0,-n.keyPanSpeed),t=!0;break;case n.keys.LEFT:e.ctrlKey||e.metaKey||e.shiftKey?P(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):N(n.keyPanSpeed,0),t=!0;break;case n.keys.RIGHT:e.ctrlKey||e.metaKey||e.shiftKey?P(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):N(-n.keyPanSpeed,0),t=!0}t&&(e.preventDefault(),n.update())}(e)}function F(e){!1!==n.enabled&&e.preventDefault()}function X(e){delete T[e.pointerId];for(let t=0;t<y.length;t++)if(y[t].pointerId==e.pointerId)return void y.splice(t,1)}function Z(e){let t=T[e.pointerId];void 0===t&&(t=new Vector2,T[e.pointerId]=t),t.set(e.pageX,e.pageY)}function B(e){const t=e.pointerId===y[0].pointerId?y[1]:y[0];return T[t.pointerId]}n.domElement.addEventListener("contextmenu",F),n.domElement.addEventListener("pointerdown",I),n.domElement.addEventListener("pointercancel",V),n.domElement.addEventListener("wheel",K,{passive:!1}),this.update()}}class MapControls extends OrbitControls{constructor(e,t){super(e,t),this.screenSpacePanning=!1,this.mouseButtons.LEFT=MOUSE.PAN,this.mouseButtons.RIGHT=MOUSE.ROTATE,this.touches.ONE=TOUCH.PAN,this.touches.TWO=TOUCH.DOLLY_ROTATE}}export{OrbitControls,MapControls};