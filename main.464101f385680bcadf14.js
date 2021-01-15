(self.webpackChunkinverse_kinematics_arm=self.webpackChunkinverse_kinematics_arm||[]).push([[179],{878:(n,e,t)=>{"use strict";t(654);var o=t(212),a=t(886),i=t(716);function r(){this.q=new o._fP,this.targetVec=new o.Pa4,this.effectorPos=new o.Pa4,this.effectorVec=new o.Pa4,this.linkPos=new o.Pa4,this.invLinkQ=new o._fP,this.linkScale=new o.Pa4,this.axis=new o.Pa4,this.vector=new o.Pa4}function s(){this.v1=new o.Pa4,this.v2=new o.Pa4,this.v3=new o.Pa4,this.v4=new o.Pa4,this.v5=new o.Pa4,this.v6=new o.Pa4,this.v7=new o.Pa4,this.v8=new o.Pa4,this.v9=new o.Pa4,this.v10=new o.Pa4,this.v11=new o.Pa4,this.v12=new o.Pa4,this.q1=new o._fP,this.q2=new o._fP,this.m1=new o.yGw,this.m2=new o.yGw,this.mFixedBaseLocation=new o.Pa4,this.mSolveDistanceThreshold=.001,this.mMinIterationChange=0,this.mFixedBaseMode=!0,this.ops=0,this.annealOnce=!1}r.prototype.solve=function(n,e,t,o,a){for(var i=n,r=void 0!==t?t:1,s=o,l=i[s.effector],c=e,d=s.links,h=0;h<r&&this.iterate(d,i,s,l,e,c,a);h++);},r.prototype.iterate=function(n,e,t,o,a,i,r){for(var s=Math,l=this.q,c=this.targetVec,d=this.effectorPos,h=this.effectorVec,p=this.linkPos,v=this.invLinkQ,u=this.linkScale,m=this.axis,f=this.vector,b=!1,g=n.length-1;g>=0;--g){var w=n[g].id;if(w!==g)throw Error("Please specify all links in the constraints array.");var y=e[w];if(!1===n[g].enabled)break;var x=n[g].limitation,P=n[g].rotationMin,C=n[g].rotationMax;if(y.matrixWorld.decompose(p,v,u),v.invert(),d.setFromMatrixPosition(o.matrixWorld),d.distanceTo(a)<1e-5)break;h.subVectors(d,p),h.applyQuaternion(v),h.normalize(),c.subVectors(i,p),c.applyQuaternion(v),c.normalize();var A=c.dot(h);if(A>1?A=1:A<-1&&(A=-1),!((A=s.acos(A))<1e-5)){if(void 0!==t.minAngle&&A<t.minAngle&&(A=t.minAngle),void 0!==t.maxAngle&&A>t.maxAngle&&(A=t.maxAngle),m.crossVectors(h,c),m.normalize(),l.setFromAxisAngle(m,A),y.quaternion.multiply(l),r&&void 0!==x){var k=y.quaternion.w;k>1&&(k=1);var F=s.sqrt(1-k*k);y.quaternion.set(x.x*F,x.y*F,x.z*F,k)}r&&void 0!==P&&y.rotation.setFromVector3(y.rotation.toVector3(f).max(P)),r&&void 0!==C&&y.rotation.setFromVector3(y.rotation.toVector3(f).min(C)),y.updateMatrixWorld(!0),b=!0}}return b},s.prototype.solve=function(n,e,t,o,a){var i=Number.MAX_VALUE,r=Number.MAX_VALUE,s=Number.MAX_VALUE;this.mFixedBaseLocation=o.fixedBaseLocation,this.ops=0;var l,c,d=this.m1,h=n[0].parent;if(!h)throw Error("Bone chain must be attached to a SkinnedMesh.");d.copy(h.matrixWorld).invert();for(var p=this.q1,v=this.v1,u=this.v2,m=o.chainProxy,f=0;f<m.length;++f)n[f].matrixWorld.decompose(v,p,u),m[f].copy(v);if(this.annealOnce)for(var b=o.chainProxy,g=o.boneLengths,w=0;w<b.length;++w){var y=g[w],x=b[w],P=b[w+1];0===w?(x.set(0,0,0),P.set(0,y,0)):P&&P.set(0,y+x.y,0)}for(c=0;c<t;++c){if((l=this.iterate(e,n,o,a,c))<i){if(i=l,l<=this.mSolveDistanceThreshold)break}else if(Math.abs(l-r)<this.mMinIterationChange)break;r=l}s=i;for(var C=o.line,A=o.chainProxy,k=C.geometry.attributes.position.array,F=0;F<A.length;++F)k[3*F]=A[F].x,k[3*F+1]=A[F].y,k[3*F+2]=A[F].z;return C.geometry.attributes.position.needsUpdate=!0,this.recomputeChainQuaternions(n,o),s},s.prototype.recomputeChainQuaternions=function(n,e){for(var t=e.chainProxy,a=new o.Pa4,i=new o.Pa4,r=new o._fP,s=new o._fP,l=new o.Pa4,c=new o.Pa4,d=new o.Pa4,h=new o.Pa4,p=0;p<n.length-1;++p){var v=n[p],u=t[p],m=t[p+1];if(0===p?a.set(0,1,0):a.copy(i),i.copy(m),i.addScaledVector(u,-1),i.normalize(),0===p)r.setFromUnitVectors(a,i),v.setRotationFromQuaternion(r);else{n[p-1].matrixWorld.decompose(l,s,c),d.copy(a),h.copy(i);var f=d.dot(h);f>1?f=1:f<-1&&(f=-1),f=Math.acos(f),d.crossVectors(d,h),d.normalize(),d.applyQuaternion(s.conjugate()),r.setFromAxisAngle(d,f),v.setRotationFromQuaternion(r)}}},s.prototype.iterate=function(n,e,t,o,a){if(e.length<1)throw Error("0 bones");this.forward(n,e,t,o,a),this.backward(n,e,t,o,a);var i=e.length-1;return t.chainProxy[i].distanceTo(n)};s.prototype.forward=function(n,e,t,o,a){var i=this.v1,r=this.v3,s=this.v2,l=this.v5,c=this.v4,d=this.v6,h=this.v7,p=this.q1,v=this.q2,u=t.boneLengths,m=t.chainProxy,f=e.length-1,b=t.links;a>=0&&this.recomputeChainQuaternions(e,t);for(var g=f-1;g>=0;--g){var w=m[g];i.copy(w);var y,x=u[g],P=b[g];if(P.id!==g)throw Error("Please specify all links in the constraints array.");var C=P.limitation,A=P.rotationMin,k=P.rotationMax;if(y=o?C?1:0:-1,g!==f-1)r.copy(m[g+1]),l.copy(r).addScaledVector(i,-1).normalize().negate(),0===y?(s.copy(m[g+2]),c.copy(s).addScaledVector(r,-1).normalize().negate(),this.applyBallConstraint(l,c,v,A,k)):2===y||1===y&&(g>0?(this.computeLocalTransform(p,g,e),d.copy(C).applyQuaternion(p)):d.copy(C),l.projectOnPlane(d).normalize()),h.copy(r).addScaledVector(l,x),w.copy(h);else if(g===f-1){var F=m[g+1];switch(F.copy(n),r.copy(F),l.copy(r).addScaledVector(i,-1).normalize().negate(),y){case 0:case 2:break;case 1:this.computeLocalTransform(p,g,e),d.copy(C).applyQuaternion(p),l.projectOnPlane(d).normalize()}h.copy(n).addScaledVector(l,x),w.copy(h)}}},s.prototype.backward=function(n,e,t,o,a){var i,r=this.v1,s=this.v3,l=this.v5,c=this.v6,d=this.v7,h=this.v2,p=this.v4,v=this.v8,u=this.v9,m=this.v10,f=this.q1,b=this.q2,g=t.chainProxy,w=t.boneLengths,y=e.length-1,x=t.links[0];if(0!==x.id)throw Error("Please specify all links in the constraints array.");var P=x.limitation,C=x.rotationMin,A=x.rotationMax;i=o&&P?1:0,a>=0&&this.recomputeChainQuaternions(e,t);for(var k=0;k<y;++k){var F=g[k],M=w[k];if(r.copy(F),0!==k){var U=g[k-1];s.copy(U);var z=g[k+1];d.copy(z),l.copy(d).addScaledVector(r,-1).normalize(),c.copy(r).addScaledVector(s,-1).normalize();var B,S=t.links[k];if(S.id!==k)throw Error("Please specify all links in the constraints array.");var D=S.limitation,I=S.rotationMin,V=S.rotationMax;if(0==(B=o?D?1:0:-1))this.applyBallConstraint(l,h,b,I,V);else if(2===B);else if(1===B){var _=g[k-1];h.copy(r).addScaledVector(_,-1).normalize(),this.computeLocalTransform(f,k,e),p.copy(D).applyQuaternion(f),l.projectOnPlane(p).normalize(),void 0===I&&void 0===V||(v.set(0,1,0),u.copy(v).applyQuaternion(f).normalize(),this.applyBallConstraint(l,u,b,I,V))}m.copy(r).addScaledVector(l,M),z.copy(m)}else{var K=g[k+1];if(d.copy(K),this.mFixedBaseMode?F.copy(this.mFixedBaseLocation):(l.copy(d).addScaledVector(r,-1).normalize(),F.copy(d).addScaledVector(l,-M)),r.copy(F),0===i)l.copy(d).addScaledVector(r,-1).normalize(),m.copy(F).addScaledVector(l,M),K.copy(m);else if(2===i);else if(1===i){var R=P;l.copy(d).addScaledVector(r,-1).normalize(),l.projectOnPlane(R).normalize(),(C||A)&&(v.set(0,1,0),P.distanceTo(v)<.001&&v.set(1,0,0),u.copy(v).cross(R).normalize(),this.applyBallConstraint(l,u,b,C,A)),m.copy(F).addScaledVector(l,M),K.copy(m)}}}},s.prototype.applyBallConstraint=function(n,e,t,a,i){var r=Math.acos(e.dot(n)),s=Math.PI;if(a&&(s=-Math.max(a.x,a.y,a.z)),i&&(s=Math.min(s,i.x,i.y,i.z)),Math.abs(r)>s){var l=new o.Pa4;l.crossVectors(e,n).normalize(),t.setFromAxisAngle(l,-Math.sign(r)*(Math.abs(r)-s)),n.applyQuaternion(t)}},s.prototype.computeLocalTransform=function(n,e,t){var o=this.m1,a=this.m2,i=this.v11,r=this.v12;a.multiplyMatrices(o,t[e-1].matrixWorld),a.decompose(i,n,r)};var l="CCD",c="FABRIK",d=function(){this.ccdSolver=new r,this.fabrikSolver=new s};function h(n){return(h="function"==typeof Symbol&&"symbol"==typeof Symbol.iterator?function(n){return typeof n}:function(n){return n&&"function"==typeof Symbol&&n.constructor===Symbol&&n!==Symbol.prototype?"symbol":typeof n})(n)}function p(n,e){for(var t=0;t<e.length;t++){var o=e[t];o.enumerable=o.enumerable||!1,o.configurable=!0,"value"in o&&(o.writable=!0),Object.defineProperty(n,o.key,o)}}function v(n,e,t){return(v="undefined"!=typeof Reflect&&Reflect.get?Reflect.get:function(n,e,t){var o=function(n,e){for(;!Object.prototype.hasOwnProperty.call(n,e)&&null!==(n=f(n)););return n}(n,e);if(o){var a=Object.getOwnPropertyDescriptor(o,e);return a.get?a.get.call(t):a.value}})(n,e,t||n)}function u(n,e){return(u=Object.setPrototypeOf||function(n,e){return n.__proto__=e,n})(n,e)}function m(n,e){return!e||"object"!==h(e)&&"function"!=typeof e?function(n){if(void 0===n)throw new ReferenceError("this hasn't been initialised - super() hasn't been called");return n}(n):e}function f(n){return(f=Object.setPrototypeOf?Object.getPrototypeOf:function(n){return n.__proto__||Object.getPrototypeOf(n)})(n)}d.prototype.customSolve=function(n,e,t){this.solve(c,n,e,3,null),this.solve(l,n,e,3,t)},d.prototype.solve=function(n,e,t,o,a,i){switch(n){case l:this.ccdSolver.solve(e,t,o,a,i);break;case c:this.fabrikSolver.solve(e,t,o,a,i)}},t(875);var b,g,w=new o.Pa4,y=new o.yGw,x=new o.yGw,P=new o.Pa4,C=new o.Pa4,A=new o.Pa4,k=new o._fP,F=new o._fP,M=new o.Pa4(0,0,1),U=new o.Pa4,z=new o.Pa4,B=function(n){!function(n,e){if("function"!=typeof e&&null!==e)throw new TypeError("Super expression must either be null or a function");n.prototype=Object.create(e&&e.prototype,{constructor:{value:n,writable:!0,configurable:!0}}),e&&u(n,e)}(s,n);var e,t,a,i,r=(a=s,i=function(){if("undefined"==typeof Reflect||!Reflect.construct)return!1;if(Reflect.construct.sham)return!1;if("function"==typeof Proxy)return!0;try{return Date.prototype.toString.call(Reflect.construct(Date,[],(function(){}))),!0}catch(n){return!1}}(),function(){var n,e=f(a);if(i){var t=f(this).constructor;n=Reflect.construct(e,arguments,t)}else n=e.apply(this,arguments);return m(this,n)});function s(n,e){var t;!function(n,e){if(!(n instanceof e))throw new TypeError("Cannot call a class as a function")}(this,s);for(var a=S(n),i=new o.u9r,l=[],c=[],d=new o.Ilk(1,0,0),h=new o.Ilk(1,0,0),p=(new o.Ilk(1,.64,0),0);p<a.length;++p){var v=a[p];v.parent&&v.parent.isBone&&(l.push(0,0,0),l.push(0,0,0),c.push(d.r,d.g,d.b),c.push(h.r,h.g,h.b))}i.setAttribute("position",new o.a$l(l,3)),i.setAttribute("color",new o.a$l(c,3));var u=new o.nls({vertexColors:!0,depthTest:!1,depthWrite:!1,toneMapped:!1,transparent:!0});(t=r.call(this,i,u)).type="IKConstraintsHelper",t.isSkeletonHelper=!0,t.root=n,t.bones=a,t.matrix=n.matrixWorld,t.matrixAutoUpdate=!1,t.constraints=e,t.boneConstraints=[];for(var m=e.links,f=m.length,b=0;b<f;++b){var g=m[b];!1!==g.enabled&&g.limitation&&(t.boneConstraints[g.id]={limitation:g.limitation,minAngle:g.minAngle,maxAngle:g.maxAngle})}var w=new o.vBJ({depthTest:!1,depthWrite:!1,transparent:!0,opacity:.8,side:o.ehD,fog:!1,toneMapped:!1,color:16776960}),y=w.clone();y.color.set(16753920);var x=new o.Tme;t.planeModel=[];for(var P=1.6,C=0;C<f;++C){var A=m[C];if(!1!==A.enabled&&A.limitation){var k=new o.BKK(P,P),F=new o.BKK(P,P),M=new o.Kj0(k,w),U=new o.Kj0(F,y);t.planeModel[A.id]=[M,U],x.add(M),x.add(U)}}return t.add(x),t}return e=s,(t=[{key:"updateMatrixWorld",value:function(n){var e=this.bones,t=this.geometry,o=t.getAttribute("position"),a=this.boneConstraints,i=this.planeModel;x.copy(this.root.matrixWorld).invert();for(var r=0,l=0;r<e.length;++r){var c=e[r],d=a[r];if(c.parent&&c.parent.isBone&&d){var h=d.limitation,p=i[r],u=p[0],m=p[1];y.multiplyMatrices(x,c.parent.matrixWorld),P.setFromMatrixPosition(y),y.decompose(A,k,U),F.setFromUnitVectors(M,h),F.premultiply(k),u.quaternion.copy(F),m.quaternion.copy(F),y.multiplyMatrices(x,c.matrixWorld),w.setFromMatrixPosition(y),o.setXYZ(l,w.x,w.y,w.z),C.copy(w),z.copy(h),z.applyQuaternion(k),u.position.copy(w).addScaledVector(z,-.2),m.position.copy(w).addScaledVector(z,.2),F.copy(k).conjugate(),y.decompose(A,k,U),w.subVectors(P,w),w.cross(z),w.normalize(),w.applyQuaternion(F).applyQuaternion(k),w.normalize(),o.setXYZ(l+1,C.x+w.x,C.y+w.y,C.z+w.z),l+=2}else c.parent&&c.parent.isBone&&(y.multiplyMatrices(x,c.parent.matrixWorld),P.setFromMatrixPosition(y),y.decompose(A,k,U),y.multiplyMatrices(x,c.matrixWorld),w.setFromMatrixPosition(y),o.setXYZ(l,w.x,w.y,w.z),C.copy(w),F.copy(k).conjugate(),y.decompose(A,k,U),w.subVectors(P,w),w.normalize(),w.applyQuaternion(k),w.normalize(),o.setXYZ(l+1,C.x+w.x,C.y+w.y,C.z+w.z),l+=2)}t.getAttribute("position").needsUpdate=!0,v(f(s.prototype),"updateMatrixWorld",this).call(this,n)}}])&&p(e.prototype,t),s}(o.ejS);function S(n){var e=[];n&&n.isBone&&e.push(n);for(var t=0;t<n.children.length;++t)e.push.apply(e,S(n.children[t]));return e}function D(n,e){var t=e.sizing,a=e.constraints,i=function(n){for(var e=new o.m_w(1,1,n.height,8,3*n.segmentCount,!0),t=e.attributes.position,a=new o.Pa4,i=[],r=[],s=0;s<t.count;s++){a.fromBufferAttribute(t,s);var l=a.y+n.halfHeight,c=Math.floor(l/n.segmentHeight),d=l%n.segmentHeight/n.segmentHeight;i.push(c,c+1,0,0),r.push(1-d,d,0,0)}e.setAttribute("skinIndex",new o.qlB(i,4)),e.setAttribute("skinWeight",new o.a$l(r,4));var h=new o.xoR({skinning:!0,color:8995605,emissive:3411975,side:o.ehD,flatShading:!0});return new o.TUv(e,h)}(t);n.add(i);var r=function(n,e,t){var a=[],i=[],r=new o.N$j;i.push(new o.Pa4(0,0,0)),a.push(r),r.position.y=-n.halfHeight;var s=new o.Pa4;s.copy(r.position);for(var l=0;l<n.segmentCount;l++){var c=new o.N$j;c.position.y=n.segmentHeight,a.push(c),r.add(c),r=c,i.push(new o.Pa4(0,(l+1)*n.segmentHeight,0))}for(var d=[],h=0;h<a.length-1;++h)d.push(n.segmentHeight);d.push(0),e.boneLengths=d,e.chainProxy=i,e.fixedBaseLocation=s;var p=new o.OdW(a);p.constraints=e,p.chainProxy=i;var v=p.bones[0];return t.add(v),t.bind(p),p}(t,a,i),s=new o._YX(i);s.material.linewidth=2,n.add(s);for(var l=new o.nls({vertexColors:!0}),c=(new o.u9r).setFromPoints(r.chainProxy),d=[],h=new o.Ilk(0,0,1),p=new o.Ilk(1,.75,0),v=0;v<r.chainProxy.length;++v)d.push(h.r,h.g,h.b),d.push(p.r,p.g,p.b);c.setAttribute("color",new o.a$l(d,3));var u=new o.x12(c,l);return a.line=u,function(n,e,t){var o=new B(n,e);t.add(o)}(i,a,n),r}var I,V=0,_=!1,K=new o.FM8,R=[],T=[],j=[],E=[],W=[],O=[],q=[],L=[],H=[],Q=[],Y=[];function N(n){var e={effector:4,links:[{id:0},{id:1},{id:2,limitation:new o.Pa4(0,0,1)},{id:3}],minAngle:0,maxAngle:1};n.constraints=e}function X(n){var e={effector:4,links:[{id:0},{id:1},{id:2,limitation:new o.Pa4(0,0,1)},{id:3,limitation:new o.Pa4(1,0,0),rotationMin:new o.Pa4(-Math.PI/2,-Math.PI/2,-Math.PI/2),rotationMax:new o.Pa4(Math.PI/2,Math.PI/2,Math.PI/2)}],minAngle:0,maxAngle:1};n.constraints=e}function J(n){n.constraints={effector:4,links:[{id:0},{id:1},{id:2},{id:3}],minAngle:0,maxAngle:1}}function $(n){n.sizing={segmentHeight:5,segmentCount:4,height:20,halfHeight:10}}function G(){I.width=window.innerWidth,I.height=window.innerHeight,Z=!1,nn()}var Z=!1;function nn(){if(!Z){I.width=window.innerWidth,I.height=window.innerHeight;var n=I.clientWidth,e=I.clientHeight;I.width===n&&I.height===e||(b.setSize(n,e,!1),g.setSize(n,e,!1)),Z=!0}}function en(n){V+=.01;for(var e=0;e<n.bones.length;e++)n.bones[e].rotation.z=2*Math.sin(V+e)/n.bones.length,n.bones[e].rotation.y=2*Math.sin(V+e)/n.bones.length}function tn(n){var e=n.clientX,t=n.clientY;K.x=e/window.innerWidth*2-1,K.y=-t/window.innerHeight*2+1;for(var a=0;a<9;++a){var i=R[a];if(i){var r=H[a],s=E[a];if(s){var l=j[a],c=O[a],d=Y[a].getBoundingClientRect();if(!(d.bottom<0||d.top>b.domElement.clientHeight||d.right<0||d.left>b.domElement.clientWidth||e<d.left||e>d.right||t<d.top||t>d.bottom||d.bottom===d.top||d.left===d.right)){var h=(e-d.left)/(d.right-d.left)*2-1,p=(d.top-t)/(d.bottom-d.top)*2+1;c.set(h,p);var v=new o.iMs;v.setFromCamera(c,i);var u=v.intersectObjects([l]);if(u.length&&u[0]){if(r.copy(u[0].point),s.position.copy(r),Q[a]=!0,s.userData.binding){var m=s.userData.binding.id,f=s.userData.binding.helper;H[m].copy(r),Q[m]=!0,f.position.copy(r)}_=!0}}}}}}function on(n,e,t,o){var a=n.bones,i=n.constraints;if(_){var r=o.constrained;switch(o.method){case 4:var s=o.iterations_hybrid[0],d=o.iterations_hybrid[1];e.solve(c,a,t,s,i,!1),e.solve(l,a,t,d,i,r);break;case 2:var h=o.iterations_ccd;e.solve(l,a,t,h,i,r);break;case 3:var p=o.iterations_fabrik;e.solve(c,a,t,p,i,r)}}}var an=document.createElement("div");an.innerHTML='\n\n<div id="info">\n    <h2>\n        Inverse Kinematics algorithms — WebGL\n    </h2>\n</div>\n<br/>\n<p>\n    The Inverse Kinematics (IK) problem is the problem of finding a\n    robotic arm’s configuration in order for this arm’s end point to reach a target.\n</p>\n\n\x3c!-- Forward demo --\x3e\n<div class="view" id="demo-0">\n</div>\n<p class="legend">A 4-bones robotic arm that does not know how to reach that orange cube.</p>\n<br/>\n\n<p>\n    While it is easy to compute an analytical solution to the IK problem when the arm\n    only contains two bones, it quickly becomes impractical as the number of bones increases.\n    There are multiple ways, however, to find a numerical,\n    approximate solution with arbitrary many bones.\n</p>\n\n<p>\n    One first, very straightforward method is called <b>CCD</b> (Cyclic Coordinate Descent):\n</p>\n<br/>\n<div class="algo">\n    <code>for <var>bone</var>: <var>last</var> &rarr; <var>first</var></code>\n    <br/>\n    <div class="indent-1">\n        <code><var>v<sub>l</sub></var> = vector(<var>bone.base</var>, <var>last.end</var>)</code>\n        <br/>\n        <code><var>v<sub>t</sub></var> = vector(<var>bone.base</var>, <var>target.end</var>)</code>\n        <br/>\n        <code><var>&alpha;</var> = angle(<var>v<sub>l</sub></var>, <var>v<sub>t</sub></var>)</code>\n        <br/>\n        <code><var>n</var> = cross(<var>v<sub>l</sub></var>, <var>v<sub>t</sub></var>)</code>\n        <br/>\n        <code>rotate <var>bone</var> around <var>bone.base</var> along\n            <var>n</var> by <var>&alpha;</var></code>\n    </div>\n    <code>repeat</code>\n    <br/>\n</div>\n<p class="legend">Algorithm: one iteration of CCD.</p>\n<br/>\n\n\x3c!-- CCD demo --\x3e\n<div class="view" id="demo-1">\n</div>\n<p class="legend">Move the cube around with the mouse to\n    see CCD in action. <small>[10 iter.]</small></p>\n<br/>\n\n<p>\n    As a matter of fact, and as you can see from the demo, CCD runs pretty fast,\n    gives smooth transitions as the target moves around, and also happens to be easy\n    to implement.\n\n    On the demo, it also gives some twist to the robotic arm:\n    if you repeatedly move the target in circles,\n    the arm will eventually get twisted as a helix\n    (although this problem can be easily solved).\n</p>\n<p>\n    This is because it works in quaternion space (~i.e. just works with angles),\n    and only looks at the quaternions from the last found solution.\n    This is a local process.\n    A solution to that problem would be to add constraints to the\n    joint angles, as we will see in just a bit.\n</p>\n<p>\n    Another simple, well-documented method is <b>FABRIK</b>\n    (Forward And Backward Reaching Inverse Kinematics):\n</p>\n<br/>\n<div class="algo">\n    <code>// forward pass</code>\n    <br/>\n    <code><var>v<sub>l</sub></var> = unit_vector(<var>last.base</var>,\n        <var>target</var>) &times; last.length</code>\n    <br/>\n    <code>snap <var>last.end</var> to <var>target</var> </code>\n    <br/>\n    <code>snap <var>last.base</var> to <var>last.end</var> -\n        <var>v<sub>l</sub></var> </code>\n    <br/>\n    <code>for <var>bone</var>: <var>last - 1</var> &rarr;\n        <var>first</var></code>\n    <br/>\n    <div class="indent-1">\n        <code><var>next</var> = <var>bone.next</var></code>\n        <br/>\n        <code><var>u<sub>b</sub></var> = unit_vector(<var>bone.base</var>,\n            <var>next.base</var>)</code>\n        <br/>\n        <code>snap <var>bone.end</var> to <var>next.base</var></code>\n        <br/>\n        <code>snap <var>bone.base</var> to <var>bone.end</var> -\n            <var>u<sub>b</sub></var> &times; bone.length</code>\n        <br/>\n    </div>\n    <br/>\n    <code>// backward pass</code>\n    <br/>\n    <code><var>v<sub>f</sub></var> = vector(<var>first.base</var>,\n        <var>first.end</var>)</code>\n    <br/>\n    <code>snap <var>first.base</var> to the robotic arm root</code>\n    <br/>\n    <code>snap <var>first.end</var> to <var>first.base</var> +\n        <var>v<sub>f</sub></var> </code>\n    <br/>\n    <code>for <var>bone</var>: <var>first + 1</var> &rarr; <var>last</var></code>\n    <br/>\n    <div class="indent-1">\n        <code><var>next</var> = <var>bone.next</var></code>\n        <br/>\n        <code><var>u<sub>b</sub></var> = unit_vector(<var>bone.base</var>,\n            <var>next.base</var>)</code>\n        <br/>\n        <code>snap <var>bone.end</var> to <var>bone.base</var> +\n            <var>u<sub>b</sub></var> &times; bone.length</code>\n        <br/>\n    </div>\n    <code>repeat</code>\n    <br/>\n</div>\n<p class="legend">Algorithm: one iteration of FABRIK.</p>\n<br/>\n\n<div class="view list-item" id="demo-2">\n</div>\n<p class="legend">FABRIK in action. <small>[10 iter.]</small></p>\n<br/>\n\n<p>\n    This method has a number of advantages over CCD.\n    It is faster as it works in vector space and does not require to compute angles,\n    and more well-behaved; all the while giving smooth results and being as easy to\n    implement as CCD.\n</p>\n\n<br/>\n<p class="legend">—</p>\n<br/>\n\n<p>\n    This could be all there is to it, but unfortunately, robotic arms (be it for robots or\n    or 3D animated skeletons)\n    are not ideal sequences of bones that can rotate just about freely.\n    In the human skeleton, for example,\n    some bones can only rotate in a certain manner or up to a certain angle.\n</p>\n\n<p>\n    This is why we need <b>constraints</b>. There are many kinds of constraints,\n    two of the most common ones are max/min angle constraints and <emph>hinges</emph>.\n    A hinge constraint simply limits the joint between two bones to operate on a single plane.\n    This is the case, for example, of the human elbow.\n</p>\n<p>\n    Fortunately, these two constraints are quite easy to include in CCD, using\n    quaternions,\n    with limited effect on its capacity to converge with very little work\n    (compared to approaches using other paradigms such as\n    the Jacobian) and to give plausible results.\n</p>\n\n<div class="view list-item" id="demo-5">\n</div>\n<p class="legend">CCD with a hinge constraint. <small>[10 iter.]</small></p>\n<br/>\n\n<p>\n    FABRIK also allows to apply hinge and angle constraints, using vector projections,\n    although the effect on the\n    convergence is somehow more noticeable.\n</p>\n\n<div class="view list-item" id="demo-6">\n</div>\n<p class="legend">FABRIK with a hinge constraint. <small>[10 iter.]</small></p>\n<br/>\n\n<p>\n    But!\n    When there are more constraints, and especially when the constraints concern bones that\n    are near the end effector, FABRIK fails to find an appropriate solution.\n</p>\n\n<div class="view-double">\n    <div class="view-left" id="demo-3">\n    </div>\n    <div class="view-right" id="demo-4">\n    </div>\n</div>\n<p class="legend">CCD <small>(left)</small> and FABRIK <small>(right)</small>\n    with two hinges constraints <br/>\n    and a max angle (&pi;/2) on the last joint. <small>[10 iter.]</small>\n</p>\n<br/>\n\n<p>\n    One of the key advantages of FABRIK is its fast long-distance convergence, and\n    a capacity to be less local, in some sense, than CCD.\n    CCD, on the other hand, displays somehow of a better\n    short-distance convergence, and most importantly better respects strict constraints.\n</p>\n\n<p>\n    Therefore, an interesting idea would be to combine the advantages of both approaches:\n</p>\n<br/>\n<div class="algo">\n    <code>drop constraints</code>\n    <br/>\n    <code>apply 2 iterations of FABRIK</code>\n    <br/>\n    <code>set constraints back</code>\n    <br/>\n    <code>apply 10 iterations of CCD</code>\n    <br/>\n</div>\n<p class="legend">Algohithm: hybrid IK approach.</p>\n<br/>\n\n<div class="view list-item" id="demo-7">\n</div>\n<p class="legend">Hybrid approach with two hinge constraints\n    <br/>\n    and a max angle (&pi;/2) on the last joint. <small>[2+10 iter.]</small>\n</p>\n<br/>\n\n<p>\n    We can see that the hybrid approach is not perfect, but\n    seems much more well-behaved than both CCD\n    (too local, less smooth, twists) and\n    FABRIK (hard time respecting constraints, vibrations).\n    There are still vibration problems with this approach,\n    especially when working with\n    longer robotic arms!\n</p>\n\n<p>\n    As FABRIK, CCD, performs locally: it can hit a local optimum\n    and never get out of it, which is very noticeable on longer arms.\n    This is because the IK promlem is non-convex, and\n    ideally heuristic methods such as FABRIK and CCD should be used along with\n    some form of\n    annealing (on the starting configurations), as advised by\n    <a href="http://number-none.com/product/IK%20with%20Quaternion%20Joint%20Limits/">Jonathan Blow</a>.\n</p>\n\n<p>\n    The annealing would become quite tedious as the skeleton grows in size, because\n    we would need to find an acceptable cover of the space of degrees of freedom\n    (see <a href="https://en.wikipedia.org/wiki/Sobol_sequence">Sobol sampling</a>),\n    and that space gets exponentially large as the number of bones increases!\n</p>\n<p>\n    Furthermore, we should make sure that transitions stay smooth…\n</p>\n\n<div class="view list-item" id="demo-8">\n</div>\n<p class="legend">FABRIK on a 20-bone arm, <br/>\n    starting from a vertical configuration.\n    <small>[10 iter.]</small></p>\n<br/>\n\n<br/>\n<br/>\n\n<p>It seems the journey to solving the IK problem in far from being over yet!</p>\n<p>Thanks for reading!</p>\n\n<br/>\n<br/>\n'.trim(),document.getElementById("content").appendChild(an),function(){I=document.getElementById("c"),(b=new o.CP7({canvas:I,antialias:!0})).setPixelRatio(window.devicePixelRatio),b.setClearColor(0,1),g=new i.M(b,{defaultColor:16777215}),window.addEventListener("mousemove",tn),window.addEventListener("resize",G,!1);for(var n=0;n<9;++n){var e={sizing:{},method:2,iterations_ccd:10,iterations_fabrik:10,iterations_hybrid:[2,10],constrained:!1,constraints:{}};$(e),J(e),W.push(e)}!function(n){var e;n[0].method=1,n[1].method=2,n[2].method=3,n[3].method=2,n[3].constrained=!0,X(n[3]),n[4].method=3,n[4].constrained=!0,X(n[4]),n[5].method=2,n[5].constrained=!0,N(n[5]),n[6].method=2,n[6].constrained=!0,N(n[6]),n[7].method=4,n[7].constrained=!0,X(n[7]),e={segmentHeight:1,segmentCount:20,height:20,halfHeight:10},n[8].sizing=e,function(n){var e={effector:20,links:new Array(20).fill(0).map((function(n,e){return{id:e}})),minAngle:0,maxAngle:1};n.constraints=e}(n[8]),n[8].method=3}(W);for(var t=0;t<9;++t){var r=document.getElementById("demo-"+t);if(r){var s=new o.xsS;s.background=new o.Ilk(0);var l=new o.cPb(45,1,1,2e3);l.position.z=40,l.position.y=30;var c=new a.z(l,r);c.enablePan=!1,s.userData.controls=c,s.userData.camera=l,s.userData.view=r;var h=new o.Mig(6710886);s.add(h);var p=new o.Ox3(8943462);p.position.set(-1,1,1).normalize(),s.add(p);var v=new o.VLJ(30,3,4473924,1118481);v.position.y=-10,s.add(v);var u=new o.BKK(30,30,2),m=new o.vBJ({color:16776960,side:o.ehD,wireframe:!0,transparent:!0,opacity:0}),f=new o.Kj0(u,m);f.rotation.x=Math.PI/2,s.add(f),s.userData.rp=f;var w=new o.VLJ(30,2);s.add(w),s.userData.gh=w;var y=new o.FM8,x=new o.nvb(1,1,1),P=new o.vBJ({color:16739885,opacity:.5}),C=new o.Kj0(x,P);s.add(C);var A=W[t],k=D(s,A),F=new d,M=new o.Pa4(0,10,0);Y.push(r),R.push(l),T.push(s),j.push(f),E.push(C),W.push(A),O.push(y),q.push(k),L.push(F),H.push(M),Q.push(!1)}}!function(n,e){E[4].userData.binding={id:3,helper:E[3]},E[3].userData.binding={id:4,helper:E[4]};var t=T[3].userData.camera,o=T[3].userData.view,i=T[4].userData.camera,r=T[4].userData.view,s=new a.z(t,r),l=new a.z(i,o);s.enablePan=!1,l.enablePan=!1}(),T[8].userData.rp.position.y+=2,T[8].userData.gh.position.y+=2,L[8].fabrikSolver.annealOnce=!0}(),function n(){!function(){nn(),b.setClearColor(0),b.setScissorTest(!1),b.clear(),b.setClearColor(0),b.setScissorTest(!0);for(var n=0;n<9;++n){var e=T[n];if(e){var t=W[n],o=q[n],a=L[n],i=R[n],r=H[n],s=Y[n].getBoundingClientRect();if(!(s.bottom<0||s.top>b.domElement.clientHeight||s.right<0||s.left>b.domElement.clientWidth)){var l=s.right-s.left,c=s.bottom-s.top,d=s.left,h=b.domElement.clientHeight-s.bottom;b.setViewport(d,h,l,c),b.setScissor(d,h,l,c),1===t.method?en(o):0!==t.method&&Q[n]&&on(o,a,r,t),g.render(e,i)}}}Q.fill(!1),_=!1}(),requestAnimationFrame(n)}()},426:(n,e,t)=>{"use strict";t.r(e),t.d(e,{default:()=>i});var o=t(645),a=t.n(o)()((function(n){return n[1]}));a.push([n.id,"\n/* latin-ext */\n@font-face {\n    font-family: 'Raleway';\n    font-style: normal;\n    font-weight: 400;\n    src: url(https://fonts.gstatic.com/s/raleway/v18/1Ptug8zYS_SKggPNyCMIT5lu.woff2) format('woff2');\n    unicode-range: U+0100-024F, U+0259, U+1E00-1EFF, U+2020, U+20A0-20AB, U+20AD-20CF, U+2113, U+2C60-2C7F, U+A720-A7FF;\n}\n/* latin */\n@font-face {\n    font-family: 'Raleway';\n    font-style: normal;\n    font-weight: 400;\n    src: url(https://fonts.gstatic.com/s/raleway/v18/1Ptug8zYS_SKggPNyC0ITw.woff2) format('woff2');\n    unicode-range: U+0000-00FF, U+0131, U+0152-0153, U+02BB-02BC, U+02C6, U+02DA, U+02DC, U+2000-206F, U+2074, U+20AC, U+2122, U+2191, U+2193, U+2212, U+2215, U+FEFF, U+FFFD;\n}\n/* latin-ext */\n@font-face {\n    font-family: 'Raleway';\n    font-style: normal;\n    font-weight: 700;\n    src: url(https://fonts.gstatic.com/s/raleway/v18/1Ptug8zYS_SKggPNyCMIT5lu.woff2) format('woff2');\n    unicode-range: U+0100-024F, U+0259, U+1E00-1EFF, U+2020, U+20A0-20AB, U+20AD-20CF, U+2113, U+2C60-2C7F, U+A720-A7FF;\n}\n/* latin */\n@font-face {\n    font-family: 'Raleway';\n    font-style: normal;\n    font-weight: 700;\n    src: url(https://fonts.gstatic.com/s/raleway/v18/1Ptug8zYS_SKggPNyC0ITw.woff2) format('woff2');\n    unicode-range: U+0000-00FF, U+0131, U+0152-0153, U+02BB-02BC, U+02C6, U+02DA, U+02DC, U+2000-206F, U+2074, U+20AC, U+2122, U+2191, U+2193, U+2212, U+2215, U+FEFF, U+FFFD;\n}\n\n* {\n    box-sizing: border-box;\n    -moz-box-sizing: border-box;\n}\n\nbody {\n    background-color: #000;\n    color: lightgrey;\n    margin: auto;\n    padding: .5in;\n    max-width: 7in;\n    text-align: justify;\n    font-family: Raleway, sans-serif;\n}\n\na {\n    color: #08f;\n}\n\n#info {\n    left: 0;\n}\n\ncode {\n    font-size: large;\n}\n\n.algo {\n    width: 5in;\n    margin: auto;\n}\n\n.indent-1 {\n    margin-top: 5px;\n    margin-bottom: 5px;\n    margin-left: 10px;\n    padding-left: 15px;\n    border-left: 2px lightgrey solid;\n}\n\n.view, .view-double {\n    width: 5in;\n    margin: auto;\n}\n\n.view {\n    height: 5in;\n}\n\n.view-double {\n    height: 2.5in;\n    display: block;\n}\n\n.view-left, .view-right {\n    display: inline-block;\n    margin: auto;\n    width: 2.3in;\n    height: 2.5in;\n}\n\n#c {\n    position: fixed;\n    left: 0; top: 0;\n    width: 100%;\n    height: 100%;\n    background-color: #000;\n    z-index: -1;\n}\n\n#content {\n    position: absolute;\n    margin: auto;\n    /*padding: .5in;*/\n    max-width: 7in;\n    text-align: justify;\n    z-index: 1;\n}\n\n.legend {\n    font-style: italic;\n    text-align: center;\n}\n\n#info {\n    font-weight: 700;\n    text-align: center;\n}\n",""]);const i=a},654:(n,e,t)=>{"use strict";var o=t(379),a=t.n(o),i=t(426),r=a()(i.default,{insert:"head",singleton:!1});if(!i.default.locals||n.hot.invalidate){var s=i.default.locals;n.hot.accept(426,(e=>{i=t(426),function(n,e,t){if(!n&&e||n&&!e)return!1;var o;for(o in n)if(n[o]!==e[o])return!1;for(o in e)if(!n[o])return!1;return!0}(s,i.default.locals)?(s=i.default.locals,r(i.default)):n.hot.invalidate()}))}n.hot.dispose((function(){r()})),i.default.locals}},0,[[878,666,216]]]);