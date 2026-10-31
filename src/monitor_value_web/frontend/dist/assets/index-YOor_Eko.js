(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const o of document.querySelectorAll('link[rel="modulepreload"]'))r(o);new MutationObserver(o=>{for(const u of o)if(u.type==="childList")for(const c of u.addedNodes)c.tagName==="LINK"&&c.rel==="modulepreload"&&r(c)}).observe(document,{childList:!0,subtree:!0});function t(o){const u={};return o.integrity&&(u.integrity=o.integrity),o.referrerPolicy&&(u.referrerPolicy=o.referrerPolicy),o.crossOrigin==="use-credentials"?u.credentials="include":o.crossOrigin==="anonymous"?u.credentials="omit":u.credentials="same-origin",u}function r(o){if(o.ep)return;o.ep=!0;const u=t(o);fetch(o.href,u)}})();function Qv(s){return s&&s.__esModule&&Object.prototype.hasOwnProperty.call(s,"default")?s.default:s}var $c={exports:{}},ja={},Kc={exports:{}},xt={};/**
 * @license React
 * react.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var lm;function Jv(){if(lm)return xt;lm=1;var s=Symbol.for("react.element"),e=Symbol.for("react.portal"),t=Symbol.for("react.fragment"),r=Symbol.for("react.strict_mode"),o=Symbol.for("react.profiler"),u=Symbol.for("react.provider"),c=Symbol.for("react.context"),f=Symbol.for("react.forward_ref"),h=Symbol.for("react.suspense"),m=Symbol.for("react.memo"),g=Symbol.for("react.lazy"),y=Symbol.iterator;function v(k){return k===null||typeof k!="object"?null:(k=y&&k[y]||k["@@iterator"],typeof k=="function"?k:null)}var M={isMounted:function(){return!1},enqueueForceUpdate:function(){},enqueueReplaceState:function(){},enqueueSetState:function(){}},T=Object.assign,S={};function x(k,ce,ke){this.props=k,this.context=ce,this.refs=S,this.updater=ke||M}x.prototype.isReactComponent={},x.prototype.setState=function(k,ce){if(typeof k!="object"&&typeof k!="function"&&k!=null)throw Error("setState(...): takes an object of state variables to update or a function which returns an object of state variables.");this.updater.enqueueSetState(this,k,ce,"setState")},x.prototype.forceUpdate=function(k){this.updater.enqueueForceUpdate(this,k,"forceUpdate")};function _(){}_.prototype=x.prototype;function P(k,ce,ke){this.props=k,this.context=ce,this.refs=S,this.updater=ke||M}var R=P.prototype=new _;R.constructor=P,T(R,x.prototype),R.isPureReactComponent=!0;var L=Array.isArray,Y=Object.prototype.hasOwnProperty,U={current:null},N={key:!0,ref:!0,__self:!0,__source:!0};function H(k,ce,ke){var J,de={},Se=null,Me=null;if(ce!=null)for(J in ce.ref!==void 0&&(Me=ce.ref),ce.key!==void 0&&(Se=""+ce.key),ce)Y.call(ce,J)&&!N.hasOwnProperty(J)&&(de[J]=ce[J]);var Ie=arguments.length-2;if(Ie===1)de.children=ke;else if(1<Ie){for(var Ge=Array(Ie),Qe=0;Qe<Ie;Qe++)Ge[Qe]=arguments[Qe+2];de.children=Ge}if(k&&k.defaultProps)for(J in Ie=k.defaultProps,Ie)de[J]===void 0&&(de[J]=Ie[J]);return{$$typeof:s,type:k,key:Se,ref:Me,props:de,_owner:U.current}}function b(k,ce){return{$$typeof:s,type:k.type,key:ce,ref:k.ref,props:k.props,_owner:k._owner}}function w(k){return typeof k=="object"&&k!==null&&k.$$typeof===s}function I(k){var ce={"=":"=0",":":"=2"};return"$"+k.replace(/[=:]/g,function(ke){return ce[ke]})}var K=/\/+/g;function V(k,ce){return typeof k=="object"&&k!==null&&k.key!=null?I(""+k.key):ce.toString(36)}function Z(k,ce,ke,J,de){var Se=typeof k;(Se==="undefined"||Se==="boolean")&&(k=null);var Me=!1;if(k===null)Me=!0;else switch(Se){case"string":case"number":Me=!0;break;case"object":switch(k.$$typeof){case s:case e:Me=!0}}if(Me)return Me=k,de=de(Me),k=J===""?"."+V(Me,0):J,L(de)?(ke="",k!=null&&(ke=k.replace(K,"$&/")+"/"),Z(de,ce,ke,"",function(Qe){return Qe})):de!=null&&(w(de)&&(de=b(de,ke+(!de.key||Me&&Me.key===de.key?"":(""+de.key).replace(K,"$&/")+"/")+k)),ce.push(de)),1;if(Me=0,J=J===""?".":J+":",L(k))for(var Ie=0;Ie<k.length;Ie++){Se=k[Ie];var Ge=J+V(Se,Ie);Me+=Z(Se,ce,ke,Ge,de)}else if(Ge=v(k),typeof Ge=="function")for(k=Ge.call(k),Ie=0;!(Se=k.next()).done;)Se=Se.value,Ge=J+V(Se,Ie++),Me+=Z(Se,ce,ke,Ge,de);else if(Se==="object")throw ce=String(k),Error("Objects are not valid as a React child (found: "+(ce==="[object Object]"?"object with keys {"+Object.keys(k).join(", ")+"}":ce)+"). If you meant to render a collection of children, use an array instead.");return Me}function te(k,ce,ke){if(k==null)return k;var J=[],de=0;return Z(k,J,"","",function(Se){return ce.call(ke,Se,de++)}),J}function ue(k){if(k._status===-1){var ce=k._result;ce=ce(),ce.then(function(ke){(k._status===0||k._status===-1)&&(k._status=1,k._result=ke)},function(ke){(k._status===0||k._status===-1)&&(k._status=2,k._result=ke)}),k._status===-1&&(k._status=0,k._result=ce)}if(k._status===1)return k._result.default;throw k._result}var se={current:null},G={transition:null},he={ReactCurrentDispatcher:se,ReactCurrentBatchConfig:G,ReactCurrentOwner:U};function fe(){throw Error("act(...) is not supported in production builds of React.")}return xt.Children={map:te,forEach:function(k,ce,ke){te(k,function(){ce.apply(this,arguments)},ke)},count:function(k){var ce=0;return te(k,function(){ce++}),ce},toArray:function(k){return te(k,function(ce){return ce})||[]},only:function(k){if(!w(k))throw Error("React.Children.only expected to receive a single React element child.");return k}},xt.Component=x,xt.Fragment=t,xt.Profiler=o,xt.PureComponent=P,xt.StrictMode=r,xt.Suspense=h,xt.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=he,xt.act=fe,xt.cloneElement=function(k,ce,ke){if(k==null)throw Error("React.cloneElement(...): The argument must be a React element, but you passed "+k+".");var J=T({},k.props),de=k.key,Se=k.ref,Me=k._owner;if(ce!=null){if(ce.ref!==void 0&&(Se=ce.ref,Me=U.current),ce.key!==void 0&&(de=""+ce.key),k.type&&k.type.defaultProps)var Ie=k.type.defaultProps;for(Ge in ce)Y.call(ce,Ge)&&!N.hasOwnProperty(Ge)&&(J[Ge]=ce[Ge]===void 0&&Ie!==void 0?Ie[Ge]:ce[Ge])}var Ge=arguments.length-2;if(Ge===1)J.children=ke;else if(1<Ge){Ie=Array(Ge);for(var Qe=0;Qe<Ge;Qe++)Ie[Qe]=arguments[Qe+2];J.children=Ie}return{$$typeof:s,type:k.type,key:de,ref:Se,props:J,_owner:Me}},xt.createContext=function(k){return k={$$typeof:c,_currentValue:k,_currentValue2:k,_threadCount:0,Provider:null,Consumer:null,_defaultValue:null,_globalName:null},k.Provider={$$typeof:u,_context:k},k.Consumer=k},xt.createElement=H,xt.createFactory=function(k){var ce=H.bind(null,k);return ce.type=k,ce},xt.createRef=function(){return{current:null}},xt.forwardRef=function(k){return{$$typeof:f,render:k}},xt.isValidElement=w,xt.lazy=function(k){return{$$typeof:g,_payload:{_status:-1,_result:k},_init:ue}},xt.memo=function(k,ce){return{$$typeof:m,type:k,compare:ce===void 0?null:ce}},xt.startTransition=function(k){var ce=G.transition;G.transition={};try{k()}finally{G.transition=ce}},xt.unstable_act=fe,xt.useCallback=function(k,ce){return se.current.useCallback(k,ce)},xt.useContext=function(k){return se.current.useContext(k)},xt.useDebugValue=function(){},xt.useDeferredValue=function(k){return se.current.useDeferredValue(k)},xt.useEffect=function(k,ce){return se.current.useEffect(k,ce)},xt.useId=function(){return se.current.useId()},xt.useImperativeHandle=function(k,ce,ke){return se.current.useImperativeHandle(k,ce,ke)},xt.useInsertionEffect=function(k,ce){return se.current.useInsertionEffect(k,ce)},xt.useLayoutEffect=function(k,ce){return se.current.useLayoutEffect(k,ce)},xt.useMemo=function(k,ce){return se.current.useMemo(k,ce)},xt.useReducer=function(k,ce,ke){return se.current.useReducer(k,ce,ke)},xt.useRef=function(k){return se.current.useRef(k)},xt.useState=function(k){return se.current.useState(k)},xt.useSyncExternalStore=function(k,ce,ke){return se.current.useSyncExternalStore(k,ce,ke)},xt.useTransition=function(){return se.current.useTransition()},xt.version="18.3.1",xt}var um;function Sd(){return um||(um=1,Kc.exports=Jv()),Kc.exports}/**
 * @license React
 * react-jsx-runtime.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var cm;function e0(){if(cm)return ja;cm=1;var s=Sd(),e=Symbol.for("react.element"),t=Symbol.for("react.fragment"),r=Object.prototype.hasOwnProperty,o=s.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.ReactCurrentOwner,u={key:!0,ref:!0,__self:!0,__source:!0};function c(f,h,m){var g,y={},v=null,M=null;m!==void 0&&(v=""+m),h.key!==void 0&&(v=""+h.key),h.ref!==void 0&&(M=h.ref);for(g in h)r.call(h,g)&&!u.hasOwnProperty(g)&&(y[g]=h[g]);if(f&&f.defaultProps)for(g in h=f.defaultProps,h)y[g]===void 0&&(y[g]=h[g]);return{$$typeof:e,type:f,key:v,ref:M,props:y,_owner:o.current}}return ja.Fragment=t,ja.jsx=c,ja.jsxs=c,ja}var fm;function t0(){return fm||(fm=1,$c.exports=e0()),$c.exports}var O=t0(),Ke=Sd();const n0=Qv(Ke);var hl={},Zc={exports:{}},Pn={},Qc={exports:{}},Jc={};/**
 * @license React
 * scheduler.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var dm;function i0(){return dm||(dm=1,(function(s){function e(G,he){var fe=G.length;G.push(he);e:for(;0<fe;){var k=fe-1>>>1,ce=G[k];if(0<o(ce,he))G[k]=he,G[fe]=ce,fe=k;else break e}}function t(G){return G.length===0?null:G[0]}function r(G){if(G.length===0)return null;var he=G[0],fe=G.pop();if(fe!==he){G[0]=fe;e:for(var k=0,ce=G.length,ke=ce>>>1;k<ke;){var J=2*(k+1)-1,de=G[J],Se=J+1,Me=G[Se];if(0>o(de,fe))Se<ce&&0>o(Me,de)?(G[k]=Me,G[Se]=fe,k=Se):(G[k]=de,G[J]=fe,k=J);else if(Se<ce&&0>o(Me,fe))G[k]=Me,G[Se]=fe,k=Se;else break e}}return he}function o(G,he){var fe=G.sortIndex-he.sortIndex;return fe!==0?fe:G.id-he.id}if(typeof performance=="object"&&typeof performance.now=="function"){var u=performance;s.unstable_now=function(){return u.now()}}else{var c=Date,f=c.now();s.unstable_now=function(){return c.now()-f}}var h=[],m=[],g=1,y=null,v=3,M=!1,T=!1,S=!1,x=typeof setTimeout=="function"?setTimeout:null,_=typeof clearTimeout=="function"?clearTimeout:null,P=typeof setImmediate<"u"?setImmediate:null;typeof navigator<"u"&&navigator.scheduling!==void 0&&navigator.scheduling.isInputPending!==void 0&&navigator.scheduling.isInputPending.bind(navigator.scheduling);function R(G){for(var he=t(m);he!==null;){if(he.callback===null)r(m);else if(he.startTime<=G)r(m),he.sortIndex=he.expirationTime,e(h,he);else break;he=t(m)}}function L(G){if(S=!1,R(G),!T)if(t(h)!==null)T=!0,ue(Y);else{var he=t(m);he!==null&&se(L,he.startTime-G)}}function Y(G,he){T=!1,S&&(S=!1,_(H),H=-1),M=!0;var fe=v;try{for(R(he),y=t(h);y!==null&&(!(y.expirationTime>he)||G&&!I());){var k=y.callback;if(typeof k=="function"){y.callback=null,v=y.priorityLevel;var ce=k(y.expirationTime<=he);he=s.unstable_now(),typeof ce=="function"?y.callback=ce:y===t(h)&&r(h),R(he)}else r(h);y=t(h)}if(y!==null)var ke=!0;else{var J=t(m);J!==null&&se(L,J.startTime-he),ke=!1}return ke}finally{y=null,v=fe,M=!1}}var U=!1,N=null,H=-1,b=5,w=-1;function I(){return!(s.unstable_now()-w<b)}function K(){if(N!==null){var G=s.unstable_now();w=G;var he=!0;try{he=N(!0,G)}finally{he?V():(U=!1,N=null)}}else U=!1}var V;if(typeof P=="function")V=function(){P(K)};else if(typeof MessageChannel<"u"){var Z=new MessageChannel,te=Z.port2;Z.port1.onmessage=K,V=function(){te.postMessage(null)}}else V=function(){x(K,0)};function ue(G){N=G,U||(U=!0,V())}function se(G,he){H=x(function(){G(s.unstable_now())},he)}s.unstable_IdlePriority=5,s.unstable_ImmediatePriority=1,s.unstable_LowPriority=4,s.unstable_NormalPriority=3,s.unstable_Profiling=null,s.unstable_UserBlockingPriority=2,s.unstable_cancelCallback=function(G){G.callback=null},s.unstable_continueExecution=function(){T||M||(T=!0,ue(Y))},s.unstable_forceFrameRate=function(G){0>G||125<G?console.error("forceFrameRate takes a positive int between 0 and 125, forcing frame rates higher than 125 fps is not supported"):b=0<G?Math.floor(1e3/G):5},s.unstable_getCurrentPriorityLevel=function(){return v},s.unstable_getFirstCallbackNode=function(){return t(h)},s.unstable_next=function(G){switch(v){case 1:case 2:case 3:var he=3;break;default:he=v}var fe=v;v=he;try{return G()}finally{v=fe}},s.unstable_pauseExecution=function(){},s.unstable_requestPaint=function(){},s.unstable_runWithPriority=function(G,he){switch(G){case 1:case 2:case 3:case 4:case 5:break;default:G=3}var fe=v;v=G;try{return he()}finally{v=fe}},s.unstable_scheduleCallback=function(G,he,fe){var k=s.unstable_now();switch(typeof fe=="object"&&fe!==null?(fe=fe.delay,fe=typeof fe=="number"&&0<fe?k+fe:k):fe=k,G){case 1:var ce=-1;break;case 2:ce=250;break;case 5:ce=1073741823;break;case 4:ce=1e4;break;default:ce=5e3}return ce=fe+ce,G={id:g++,callback:he,priorityLevel:G,startTime:fe,expirationTime:ce,sortIndex:-1},fe>k?(G.sortIndex=fe,e(m,G),t(h)===null&&G===t(m)&&(S?(_(H),H=-1):S=!0,se(L,fe-k))):(G.sortIndex=ce,e(h,G),T||M||(T=!0,ue(Y))),G},s.unstable_shouldYield=I,s.unstable_wrapCallback=function(G){var he=v;return function(){var fe=v;v=he;try{return G.apply(this,arguments)}finally{v=fe}}}})(Jc)),Jc}var hm;function r0(){return hm||(hm=1,Qc.exports=i0()),Qc.exports}/**
 * @license React
 * react-dom.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var pm;function s0(){if(pm)return Pn;pm=1;var s=Sd(),e=r0();function t(n){for(var i="https://reactjs.org/docs/error-decoder.html?invariant="+n,a=1;a<arguments.length;a++)i+="&args[]="+encodeURIComponent(arguments[a]);return"Minified React error #"+n+"; visit "+i+" for the full message or use the non-minified dev environment for full errors and additional helpful warnings."}var r=new Set,o={};function u(n,i){c(n,i),c(n+"Capture",i)}function c(n,i){for(o[n]=i,n=0;n<i.length;n++)r.add(i[n])}var f=!(typeof window>"u"||typeof window.document>"u"||typeof window.document.createElement>"u"),h=Object.prototype.hasOwnProperty,m=/^[:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD][:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\-.0-9\u00B7\u0300-\u036F\u203F-\u2040]*$/,g={},y={};function v(n){return h.call(y,n)?!0:h.call(g,n)?!1:m.test(n)?y[n]=!0:(g[n]=!0,!1)}function M(n,i,a,l){if(a!==null&&a.type===0)return!1;switch(typeof i){case"function":case"symbol":return!0;case"boolean":return l?!1:a!==null?!a.acceptsBooleans:(n=n.toLowerCase().slice(0,5),n!=="data-"&&n!=="aria-");default:return!1}}function T(n,i,a,l){if(i===null||typeof i>"u"||M(n,i,a,l))return!0;if(l)return!1;if(a!==null)switch(a.type){case 3:return!i;case 4:return i===!1;case 5:return isNaN(i);case 6:return isNaN(i)||1>i}return!1}function S(n,i,a,l,d,p,E){this.acceptsBooleans=i===2||i===3||i===4,this.attributeName=l,this.attributeNamespace=d,this.mustUseProperty=a,this.propertyName=n,this.type=i,this.sanitizeURL=p,this.removeEmptyString=E}var x={};"children dangerouslySetInnerHTML defaultValue defaultChecked innerHTML suppressContentEditableWarning suppressHydrationWarning style".split(" ").forEach(function(n){x[n]=new S(n,0,!1,n,null,!1,!1)}),[["acceptCharset","accept-charset"],["className","class"],["htmlFor","for"],["httpEquiv","http-equiv"]].forEach(function(n){var i=n[0];x[i]=new S(i,1,!1,n[1],null,!1,!1)}),["contentEditable","draggable","spellCheck","value"].forEach(function(n){x[n]=new S(n,2,!1,n.toLowerCase(),null,!1,!1)}),["autoReverse","externalResourcesRequired","focusable","preserveAlpha"].forEach(function(n){x[n]=new S(n,2,!1,n,null,!1,!1)}),"allowFullScreen async autoFocus autoPlay controls default defer disabled disablePictureInPicture disableRemotePlayback formNoValidate hidden loop noModule noValidate open playsInline readOnly required reversed scoped seamless itemScope".split(" ").forEach(function(n){x[n]=new S(n,3,!1,n.toLowerCase(),null,!1,!1)}),["checked","multiple","muted","selected"].forEach(function(n){x[n]=new S(n,3,!0,n,null,!1,!1)}),["capture","download"].forEach(function(n){x[n]=new S(n,4,!1,n,null,!1,!1)}),["cols","rows","size","span"].forEach(function(n){x[n]=new S(n,6,!1,n,null,!1,!1)}),["rowSpan","start"].forEach(function(n){x[n]=new S(n,5,!1,n.toLowerCase(),null,!1,!1)});var _=/[\-:]([a-z])/g;function P(n){return n[1].toUpperCase()}"accent-height alignment-baseline arabic-form baseline-shift cap-height clip-path clip-rule color-interpolation color-interpolation-filters color-profile color-rendering dominant-baseline enable-background fill-opacity fill-rule flood-color flood-opacity font-family font-size font-size-adjust font-stretch font-style font-variant font-weight glyph-name glyph-orientation-horizontal glyph-orientation-vertical horiz-adv-x horiz-origin-x image-rendering letter-spacing lighting-color marker-end marker-mid marker-start overline-position overline-thickness paint-order panose-1 pointer-events rendering-intent shape-rendering stop-color stop-opacity strikethrough-position strikethrough-thickness stroke-dasharray stroke-dashoffset stroke-linecap stroke-linejoin stroke-miterlimit stroke-opacity stroke-width text-anchor text-decoration text-rendering underline-position underline-thickness unicode-bidi unicode-range units-per-em v-alphabetic v-hanging v-ideographic v-mathematical vector-effect vert-adv-y vert-origin-x vert-origin-y word-spacing writing-mode xmlns:xlink x-height".split(" ").forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,null,!1,!1)}),"xlink:actuate xlink:arcrole xlink:role xlink:show xlink:title xlink:type".split(" ").forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,"http://www.w3.org/1999/xlink",!1,!1)}),["xml:base","xml:lang","xml:space"].forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,"http://www.w3.org/XML/1998/namespace",!1,!1)}),["tabIndex","crossOrigin"].forEach(function(n){x[n]=new S(n,1,!1,n.toLowerCase(),null,!1,!1)}),x.xlinkHref=new S("xlinkHref",1,!1,"xlink:href","http://www.w3.org/1999/xlink",!0,!1),["src","href","action","formAction"].forEach(function(n){x[n]=new S(n,1,!1,n.toLowerCase(),null,!0,!0)});function R(n,i,a,l){var d=x.hasOwnProperty(i)?x[i]:null;(d!==null?d.type!==0:l||!(2<i.length)||i[0]!=="o"&&i[0]!=="O"||i[1]!=="n"&&i[1]!=="N")&&(T(i,a,d,l)&&(a=null),l||d===null?v(i)&&(a===null?n.removeAttribute(i):n.setAttribute(i,""+a)):d.mustUseProperty?n[d.propertyName]=a===null?d.type===3?!1:"":a:(i=d.attributeName,l=d.attributeNamespace,a===null?n.removeAttribute(i):(d=d.type,a=d===3||d===4&&a===!0?"":""+a,l?n.setAttributeNS(l,i,a):n.setAttribute(i,a))))}var L=s.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED,Y=Symbol.for("react.element"),U=Symbol.for("react.portal"),N=Symbol.for("react.fragment"),H=Symbol.for("react.strict_mode"),b=Symbol.for("react.profiler"),w=Symbol.for("react.provider"),I=Symbol.for("react.context"),K=Symbol.for("react.forward_ref"),V=Symbol.for("react.suspense"),Z=Symbol.for("react.suspense_list"),te=Symbol.for("react.memo"),ue=Symbol.for("react.lazy"),se=Symbol.for("react.offscreen"),G=Symbol.iterator;function he(n){return n===null||typeof n!="object"?null:(n=G&&n[G]||n["@@iterator"],typeof n=="function"?n:null)}var fe=Object.assign,k;function ce(n){if(k===void 0)try{throw Error()}catch(a){var i=a.stack.trim().match(/\n( *(at )?)/);k=i&&i[1]||""}return`
`+k+n}var ke=!1;function J(n,i){if(!n||ke)return"";ke=!0;var a=Error.prepareStackTrace;Error.prepareStackTrace=void 0;try{if(i)if(i=function(){throw Error()},Object.defineProperty(i.prototype,"props",{set:function(){throw Error()}}),typeof Reflect=="object"&&Reflect.construct){try{Reflect.construct(i,[])}catch(ae){var l=ae}Reflect.construct(n,[],i)}else{try{i.call()}catch(ae){l=ae}n.call(i.prototype)}else{try{throw Error()}catch(ae){l=ae}n()}}catch(ae){if(ae&&l&&typeof ae.stack=="string"){for(var d=ae.stack.split(`
`),p=l.stack.split(`
`),E=d.length-1,D=p.length-1;1<=E&&0<=D&&d[E]!==p[D];)D--;for(;1<=E&&0<=D;E--,D--)if(d[E]!==p[D]){if(E!==1||D!==1)do if(E--,D--,0>D||d[E]!==p[D]){var z=`
`+d[E].replace(" at new "," at ");return n.displayName&&z.includes("<anonymous>")&&(z=z.replace("<anonymous>",n.displayName)),z}while(1<=E&&0<=D);break}}}finally{ke=!1,Error.prepareStackTrace=a}return(n=n?n.displayName||n.name:"")?ce(n):""}function de(n){switch(n.tag){case 5:return ce(n.type);case 16:return ce("Lazy");case 13:return ce("Suspense");case 19:return ce("SuspenseList");case 0:case 2:case 15:return n=J(n.type,!1),n;case 11:return n=J(n.type.render,!1),n;case 1:return n=J(n.type,!0),n;default:return""}}function Se(n){if(n==null)return null;if(typeof n=="function")return n.displayName||n.name||null;if(typeof n=="string")return n;switch(n){case N:return"Fragment";case U:return"Portal";case b:return"Profiler";case H:return"StrictMode";case V:return"Suspense";case Z:return"SuspenseList"}if(typeof n=="object")switch(n.$$typeof){case I:return(n.displayName||"Context")+".Consumer";case w:return(n._context.displayName||"Context")+".Provider";case K:var i=n.render;return n=n.displayName,n||(n=i.displayName||i.name||"",n=n!==""?"ForwardRef("+n+")":"ForwardRef"),n;case te:return i=n.displayName||null,i!==null?i:Se(n.type)||"Memo";case ue:i=n._payload,n=n._init;try{return Se(n(i))}catch{}}return null}function Me(n){var i=n.type;switch(n.tag){case 24:return"Cache";case 9:return(i.displayName||"Context")+".Consumer";case 10:return(i._context.displayName||"Context")+".Provider";case 18:return"DehydratedFragment";case 11:return n=i.render,n=n.displayName||n.name||"",i.displayName||(n!==""?"ForwardRef("+n+")":"ForwardRef");case 7:return"Fragment";case 5:return i;case 4:return"Portal";case 3:return"Root";case 6:return"Text";case 16:return Se(i);case 8:return i===H?"StrictMode":"Mode";case 22:return"Offscreen";case 12:return"Profiler";case 21:return"Scope";case 13:return"Suspense";case 19:return"SuspenseList";case 25:return"TracingMarker";case 1:case 0:case 17:case 2:case 14:case 15:if(typeof i=="function")return i.displayName||i.name||null;if(typeof i=="string")return i}return null}function Ie(n){switch(typeof n){case"boolean":case"number":case"string":case"undefined":return n;case"object":return n;default:return""}}function Ge(n){var i=n.type;return(n=n.nodeName)&&n.toLowerCase()==="input"&&(i==="checkbox"||i==="radio")}function Qe(n){var i=Ge(n)?"checked":"value",a=Object.getOwnPropertyDescriptor(n.constructor.prototype,i),l=""+n[i];if(!n.hasOwnProperty(i)&&typeof a<"u"&&typeof a.get=="function"&&typeof a.set=="function"){var d=a.get,p=a.set;return Object.defineProperty(n,i,{configurable:!0,get:function(){return d.call(this)},set:function(E){l=""+E,p.call(this,E)}}),Object.defineProperty(n,i,{enumerable:a.enumerable}),{getValue:function(){return l},setValue:function(E){l=""+E},stopTracking:function(){n._valueTracker=null,delete n[i]}}}}function Et(n){n._valueTracker||(n._valueTracker=Qe(n))}function B(n){if(!n)return!1;var i=n._valueTracker;if(!i)return!0;var a=i.getValue(),l="";return n&&(l=Ge(n)?n.checked?"true":"false":n.value),n=l,n!==a?(i.setValue(n),!0):!1}function wt(n){if(n=n||(typeof document<"u"?document:void 0),typeof n>"u")return null;try{return n.activeElement||n.body}catch{return n.body}}function gt(n,i){var a=i.checked;return fe({},i,{defaultChecked:void 0,defaultValue:void 0,value:void 0,checked:a??n._wrapperState.initialChecked})}function ne(n,i){var a=i.defaultValue==null?"":i.defaultValue,l=i.checked!=null?i.checked:i.defaultChecked;a=Ie(i.value!=null?i.value:a),n._wrapperState={initialChecked:l,initialValue:a,controlled:i.type==="checkbox"||i.type==="radio"?i.checked!=null:i.value!=null}}function ie(n,i){i=i.checked,i!=null&&R(n,"checked",i,!1)}function De(n,i){ie(n,i);var a=Ie(i.value),l=i.type;if(a!=null)l==="number"?(a===0&&n.value===""||n.value!=a)&&(n.value=""+a):n.value!==""+a&&(n.value=""+a);else if(l==="submit"||l==="reset"){n.removeAttribute("value");return}i.hasOwnProperty("value")?Ue(n,i.type,a):i.hasOwnProperty("defaultValue")&&Ue(n,i.type,Ie(i.defaultValue)),i.checked==null&&i.defaultChecked!=null&&(n.defaultChecked=!!i.defaultChecked)}function Ae(n,i,a){if(i.hasOwnProperty("value")||i.hasOwnProperty("defaultValue")){var l=i.type;if(!(l!=="submit"&&l!=="reset"||i.value!==void 0&&i.value!==null))return;i=""+n._wrapperState.initialValue,a||i===n.value||(n.value=i),n.defaultValue=i}a=n.name,a!==""&&(n.name=""),n.defaultChecked=!!n._wrapperState.initialChecked,a!==""&&(n.name=a)}function Ue(n,i,a){(i!=="number"||wt(n.ownerDocument)!==n)&&(a==null?n.defaultValue=""+n._wrapperState.initialValue:n.defaultValue!==""+a&&(n.defaultValue=""+a))}var F=Array.isArray;function A(n,i,a,l){if(n=n.options,i){i={};for(var d=0;d<a.length;d++)i["$"+a[d]]=!0;for(a=0;a<n.length;a++)d=i.hasOwnProperty("$"+n[a].value),n[a].selected!==d&&(n[a].selected=d),d&&l&&(n[a].defaultSelected=!0)}else{for(a=""+Ie(a),i=null,d=0;d<n.length;d++){if(n[d].value===a){n[d].selected=!0,l&&(n[d].defaultSelected=!0);return}i!==null||n[d].disabled||(i=n[d])}i!==null&&(i.selected=!0)}}function oe(n,i){if(i.dangerouslySetInnerHTML!=null)throw Error(t(91));return fe({},i,{value:void 0,defaultValue:void 0,children:""+n._wrapperState.initialValue})}function ve(n,i){var a=i.value;if(a==null){if(a=i.children,i=i.defaultValue,a!=null){if(i!=null)throw Error(t(92));if(F(a)){if(1<a.length)throw Error(t(93));a=a[0]}i=a}i==null&&(i=""),a=i}n._wrapperState={initialValue:Ie(a)}}function Ee(n,i){var a=Ie(i.value),l=Ie(i.defaultValue);a!=null&&(a=""+a,a!==n.value&&(n.value=a),i.defaultValue==null&&n.defaultValue!==a&&(n.defaultValue=a)),l!=null&&(n.defaultValue=""+l)}function ge(n){var i=n.textContent;i===n._wrapperState.initialValue&&i!==""&&i!==null&&(n.value=i)}function Ze(n){switch(n){case"svg":return"http://www.w3.org/2000/svg";case"math":return"http://www.w3.org/1998/Math/MathML";default:return"http://www.w3.org/1999/xhtml"}}function Ne(n,i){return n==null||n==="http://www.w3.org/1999/xhtml"?Ze(i):n==="http://www.w3.org/2000/svg"&&i==="foreignObject"?"http://www.w3.org/1999/xhtml":n}var Be,ct=(function(n){return typeof MSApp<"u"&&MSApp.execUnsafeLocalFunction?function(i,a,l,d){MSApp.execUnsafeLocalFunction(function(){return n(i,a,l,d)})}:n})(function(n,i){if(n.namespaceURI!=="http://www.w3.org/2000/svg"||"innerHTML"in n)n.innerHTML=i;else{for(Be=Be||document.createElement("div"),Be.innerHTML="<svg>"+i.valueOf().toString()+"</svg>",i=Be.firstChild;n.firstChild;)n.removeChild(n.firstChild);for(;i.firstChild;)n.appendChild(i.firstChild)}});function Te(n,i){if(i){var a=n.firstChild;if(a&&a===n.lastChild&&a.nodeType===3){a.nodeValue=i;return}}n.textContent=i}var Oe={animationIterationCount:!0,aspectRatio:!0,borderImageOutset:!0,borderImageSlice:!0,borderImageWidth:!0,boxFlex:!0,boxFlexGroup:!0,boxOrdinalGroup:!0,columnCount:!0,columns:!0,flex:!0,flexGrow:!0,flexPositive:!0,flexShrink:!0,flexNegative:!0,flexOrder:!0,gridArea:!0,gridRow:!0,gridRowEnd:!0,gridRowSpan:!0,gridRowStart:!0,gridColumn:!0,gridColumnEnd:!0,gridColumnSpan:!0,gridColumnStart:!0,fontWeight:!0,lineClamp:!0,lineHeight:!0,opacity:!0,order:!0,orphans:!0,tabSize:!0,widows:!0,zIndex:!0,zoom:!0,fillOpacity:!0,floodOpacity:!0,stopOpacity:!0,strokeDasharray:!0,strokeDashoffset:!0,strokeMiterlimit:!0,strokeOpacity:!0,strokeWidth:!0},_t=["Webkit","ms","Moz","O"];Object.keys(Oe).forEach(function(n){_t.forEach(function(i){i=i+n.charAt(0).toUpperCase()+n.substring(1),Oe[i]=Oe[n]})});function rt(n,i,a){return i==null||typeof i=="boolean"||i===""?"":a||typeof i!="number"||i===0||Oe.hasOwnProperty(n)&&Oe[n]?(""+i).trim():i+"px"}function Ve(n,i){n=n.style;for(var a in i)if(i.hasOwnProperty(a)){var l=a.indexOf("--")===0,d=rt(a,i[a],l);a==="float"&&(a="cssFloat"),l?n.setProperty(a,d):n[a]=d}}var lt=fe({menuitem:!0},{area:!0,base:!0,br:!0,col:!0,embed:!0,hr:!0,img:!0,input:!0,keygen:!0,link:!0,meta:!0,param:!0,source:!0,track:!0,wbr:!0});function ft(n,i){if(i){if(lt[n]&&(i.children!=null||i.dangerouslySetInnerHTML!=null))throw Error(t(137,n));if(i.dangerouslySetInnerHTML!=null){if(i.children!=null)throw Error(t(60));if(typeof i.dangerouslySetInnerHTML!="object"||!("__html"in i.dangerouslySetInnerHTML))throw Error(t(61))}if(i.style!=null&&typeof i.style!="object")throw Error(t(62))}}function Ct(n,i){if(n.indexOf("-")===-1)return typeof i.is=="string";switch(n){case"annotation-xml":case"color-profile":case"font-face":case"font-face-src":case"font-face-uri":case"font-face-format":case"font-face-name":case"missing-glyph":return!1;default:return!0}}var j=null;function Ce(n){return n=n.target||n.srcElement||window,n.correspondingUseElement&&(n=n.correspondingUseElement),n.nodeType===3?n.parentNode:n}var pe=null,me=null,Re=null;function Je(n){if(n=Pa(n)){if(typeof pe!="function")throw Error(t(280));var i=n.stateNode;i&&(i=Ro(i),pe(n.stateNode,n.type,i))}}function vt(n){me?Re?Re.push(n):Re=[n]:me=n}function xe(){if(me){var n=me,i=Re;if(Re=me=null,Je(n),i)for(n=0;n<i.length;n++)Je(i[n])}}function at(n,i){return n(i)}function tt(){}var kt=!1;function qt(n,i,a){if(kt)return n(i,a);kt=!0;try{return at(n,i,a)}finally{kt=!1,(me!==null||Re!==null)&&(tt(),xe())}}function En(n,i){var a=n.stateNode;if(a===null)return null;var l=Ro(a);if(l===null)return null;a=l[i];e:switch(i){case"onClick":case"onClickCapture":case"onDoubleClick":case"onDoubleClickCapture":case"onMouseDown":case"onMouseDownCapture":case"onMouseMove":case"onMouseMoveCapture":case"onMouseUp":case"onMouseUpCapture":case"onMouseEnter":(l=!l.disabled)||(n=n.type,l=!(n==="button"||n==="input"||n==="select"||n==="textarea")),n=!l;break e;default:n=!1}if(n)return null;if(a&&typeof a!="function")throw Error(t(231,i,typeof a));return a}var Qn=!1;if(f)try{var Mi={};Object.defineProperty(Mi,"passive",{get:function(){Qn=!0}}),window.addEventListener("test",Mi,Mi),window.removeEventListener("test",Mi,Mi)}catch{Qn=!1}function oo(n,i,a,l,d,p,E,D,z){var ae=Array.prototype.slice.call(arguments,3);try{i.apply(a,ae)}catch(ye){this.onError(ye)}}var Gi=!1,Ei=null,Tr=!1,Wi=null,lo={onError:function(n){Gi=!0,Ei=n}};function uo(n,i,a,l,d,p,E,D,z){Gi=!1,Ei=null,oo.apply(lo,arguments)}function _u(n,i,a,l,d,p,E,D,z){if(uo.apply(this,arguments),Gi){if(Gi){var ae=Ei;Gi=!1,Ei=null}else throw Error(t(198));Tr||(Tr=!0,Wi=ae)}}function wi(n){var i=n,a=n;if(n.alternate)for(;i.return;)i=i.return;else{n=i;do i=n,(i.flags&4098)!==0&&(a=i.return),n=i.return;while(n)}return i.tag===3?a:null}function co(n){if(n.tag===13){var i=n.memoizedState;if(i===null&&(n=n.alternate,n!==null&&(i=n.memoizedState)),i!==null)return i.dehydrated}return null}function C(n){if(wi(n)!==n)throw Error(t(188))}function X(n){var i=n.alternate;if(!i){if(i=wi(n),i===null)throw Error(t(188));return i!==n?null:n}for(var a=n,l=i;;){var d=a.return;if(d===null)break;var p=d.alternate;if(p===null){if(l=d.return,l!==null){a=l;continue}break}if(d.child===p.child){for(p=d.child;p;){if(p===a)return C(d),n;if(p===l)return C(d),i;p=p.sibling}throw Error(t(188))}if(a.return!==l.return)a=d,l=p;else{for(var E=!1,D=d.child;D;){if(D===a){E=!0,a=d,l=p;break}if(D===l){E=!0,l=d,a=p;break}D=D.sibling}if(!E){for(D=p.child;D;){if(D===a){E=!0,a=p,l=d;break}if(D===l){E=!0,l=p,a=d;break}D=D.sibling}if(!E)throw Error(t(189))}}if(a.alternate!==l)throw Error(t(190))}if(a.tag!==3)throw Error(t(188));return a.stateNode.current===a?n:i}function re(n){return n=X(n),n!==null?le(n):null}function le(n){if(n.tag===5||n.tag===6)return n;for(n=n.child;n!==null;){var i=le(n);if(i!==null)return i;n=n.sibling}return null}var q=e.unstable_scheduleCallback,Pe=e.unstable_cancelCallback,ze=e.unstable_shouldYield,Xe=e.unstable_requestPaint,Le=e.unstable_now,ot=e.unstable_getCurrentPriorityLevel,st=e.unstable_ImmediatePriority,et=e.unstable_UserBlockingPriority,St=e.unstable_NormalPriority,Bt=e.unstable_LowPriority,It=e.unstable_IdlePriority,tn=null,dt=null;function $e(n){if(dt&&typeof dt.onCommitFiberRoot=="function")try{dt.onCommitFiberRoot(tn,n,void 0,(n.current.flags&128)===128)}catch{}}var bt=Math.clz32?Math.clz32:ji,Tt=Math.log,Nn=Math.LN2;function ji(n){return n>>>=0,n===0?32:31-(Tt(n)/Nn|0)|0}var nn=64,Xi=4194304;function Nt(n){switch(n&-n){case 1:return 1;case 2:return 2;case 4:return 4;case 8:return 8;case 16:return 16;case 32:return 32;case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:return n&4194240;case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:return n&130023424;case 134217728:return 134217728;case 268435456:return 268435456;case 536870912:return 536870912;case 1073741824:return 1073741824;default:return n}}function Dn(n,i){var a=n.pendingLanes;if(a===0)return 0;var l=0,d=n.suspendedLanes,p=n.pingedLanes,E=a&268435455;if(E!==0){var D=E&~d;D!==0?l=Nt(D):(p&=E,p!==0&&(l=Nt(p)))}else E=a&~d,E!==0?l=Nt(E):p!==0&&(l=Nt(p));if(l===0)return 0;if(i!==0&&i!==l&&(i&d)===0&&(d=l&-l,p=i&-i,d>=p||d===16&&(p&4194240)!==0))return i;if((l&4)!==0&&(l|=a&16),i=n.entangledLanes,i!==0)for(n=n.entanglements,i&=l;0<i;)a=31-bt(i),d=1<<a,l|=n[a],i&=~d;return l}function da(n,i){switch(n){case 1:case 2:case 4:return i+250;case 8:case 16:case 32:case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:return i+5e3;case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:return-1;case 134217728:case 268435456:case 536870912:case 1073741824:return-1;default:return-1}}function wn(n,i){for(var a=n.suspendedLanes,l=n.pingedLanes,d=n.expirationTimes,p=n.pendingLanes;0<p;){var E=31-bt(p),D=1<<E,z=d[E];z===-1?((D&a)===0||(D&l)!==0)&&(d[E]=da(D,i)):z<=i&&(n.expiredLanes|=D),p&=~D}}function Ar(n){return n=n.pendingLanes&-1073741825,n!==0?n:n&1073741824?1073741824:0}function fo(){var n=nn;return nn<<=1,(nn&4194240)===0&&(nn=64),n}function rs(n){for(var i=[],a=0;31>a;a++)i.push(n);return i}function ha(n,i,a){n.pendingLanes|=i,i!==536870912&&(n.suspendedLanes=0,n.pingedLanes=0),n=n.eventTimes,i=31-bt(i),n[i]=a}function x_(n,i){var a=n.pendingLanes&~i;n.pendingLanes=i,n.suspendedLanes=0,n.pingedLanes=0,n.expiredLanes&=i,n.mutableReadLanes&=i,n.entangledLanes&=i,i=n.entanglements;var l=n.eventTimes;for(n=n.expirationTimes;0<a;){var d=31-bt(a),p=1<<d;i[d]=0,l[d]=-1,n[d]=-1,a&=~p}}function vu(n,i){var a=n.entangledLanes|=i;for(n=n.entanglements;a;){var l=31-bt(a),d=1<<l;d&i|n[l]&i&&(n[l]|=i),a&=~d}}var Pt=0;function zd(n){return n&=-n,1<n?4<n?(n&268435455)!==0?16:536870912:4:1}var Hd,xu,Vd,Gd,Wd,yu=!1,ho=[],Yi=null,qi=null,$i=null,pa=new Map,ma=new Map,Ki=[],y_="mousedown mouseup touchcancel touchend touchstart auxclick dblclick pointercancel pointerdown pointerup dragend dragstart drop compositionend compositionstart keydown keypress keyup input textInput copy cut paste click change contextmenu reset submit".split(" ");function jd(n,i){switch(n){case"focusin":case"focusout":Yi=null;break;case"dragenter":case"dragleave":qi=null;break;case"mouseover":case"mouseout":$i=null;break;case"pointerover":case"pointerout":pa.delete(i.pointerId);break;case"gotpointercapture":case"lostpointercapture":ma.delete(i.pointerId)}}function ga(n,i,a,l,d,p){return n===null||n.nativeEvent!==p?(n={blockedOn:i,domEventName:a,eventSystemFlags:l,nativeEvent:p,targetContainers:[d]},i!==null&&(i=Pa(i),i!==null&&xu(i)),n):(n.eventSystemFlags|=l,i=n.targetContainers,d!==null&&i.indexOf(d)===-1&&i.push(d),n)}function S_(n,i,a,l,d){switch(i){case"focusin":return Yi=ga(Yi,n,i,a,l,d),!0;case"dragenter":return qi=ga(qi,n,i,a,l,d),!0;case"mouseover":return $i=ga($i,n,i,a,l,d),!0;case"pointerover":var p=d.pointerId;return pa.set(p,ga(pa.get(p)||null,n,i,a,l,d)),!0;case"gotpointercapture":return p=d.pointerId,ma.set(p,ga(ma.get(p)||null,n,i,a,l,d)),!0}return!1}function Xd(n){var i=Cr(n.target);if(i!==null){var a=wi(i);if(a!==null){if(i=a.tag,i===13){if(i=co(a),i!==null){n.blockedOn=i,Wd(n.priority,function(){Vd(a)});return}}else if(i===3&&a.stateNode.current.memoizedState.isDehydrated){n.blockedOn=a.tag===3?a.stateNode.containerInfo:null;return}}}n.blockedOn=null}function po(n){if(n.blockedOn!==null)return!1;for(var i=n.targetContainers;0<i.length;){var a=Mu(n.domEventName,n.eventSystemFlags,i[0],n.nativeEvent);if(a===null){a=n.nativeEvent;var l=new a.constructor(a.type,a);j=l,a.target.dispatchEvent(l),j=null}else return i=Pa(a),i!==null&&xu(i),n.blockedOn=a,!1;i.shift()}return!0}function Yd(n,i,a){po(n)&&a.delete(i)}function M_(){yu=!1,Yi!==null&&po(Yi)&&(Yi=null),qi!==null&&po(qi)&&(qi=null),$i!==null&&po($i)&&($i=null),pa.forEach(Yd),ma.forEach(Yd)}function _a(n,i){n.blockedOn===i&&(n.blockedOn=null,yu||(yu=!0,e.unstable_scheduleCallback(e.unstable_NormalPriority,M_)))}function va(n){function i(d){return _a(d,n)}if(0<ho.length){_a(ho[0],n);for(var a=1;a<ho.length;a++){var l=ho[a];l.blockedOn===n&&(l.blockedOn=null)}}for(Yi!==null&&_a(Yi,n),qi!==null&&_a(qi,n),$i!==null&&_a($i,n),pa.forEach(i),ma.forEach(i),a=0;a<Ki.length;a++)l=Ki[a],l.blockedOn===n&&(l.blockedOn=null);for(;0<Ki.length&&(a=Ki[0],a.blockedOn===null);)Xd(a),a.blockedOn===null&&Ki.shift()}var ss=L.ReactCurrentBatchConfig,mo=!0;function E_(n,i,a,l){var d=Pt,p=ss.transition;ss.transition=null;try{Pt=1,Su(n,i,a,l)}finally{Pt=d,ss.transition=p}}function w_(n,i,a,l){var d=Pt,p=ss.transition;ss.transition=null;try{Pt=4,Su(n,i,a,l)}finally{Pt=d,ss.transition=p}}function Su(n,i,a,l){if(mo){var d=Mu(n,i,a,l);if(d===null)Bu(n,i,l,go,a),jd(n,l);else if(S_(d,n,i,a,l))l.stopPropagation();else if(jd(n,l),i&4&&-1<y_.indexOf(n)){for(;d!==null;){var p=Pa(d);if(p!==null&&Hd(p),p=Mu(n,i,a,l),p===null&&Bu(n,i,l,go,a),p===d)break;d=p}d!==null&&l.stopPropagation()}else Bu(n,i,l,null,a)}}var go=null;function Mu(n,i,a,l){if(go=null,n=Ce(l),n=Cr(n),n!==null)if(i=wi(n),i===null)n=null;else if(a=i.tag,a===13){if(n=co(i),n!==null)return n;n=null}else if(a===3){if(i.stateNode.current.memoizedState.isDehydrated)return i.tag===3?i.stateNode.containerInfo:null;n=null}else i!==n&&(n=null);return go=n,null}function qd(n){switch(n){case"cancel":case"click":case"close":case"contextmenu":case"copy":case"cut":case"auxclick":case"dblclick":case"dragend":case"dragstart":case"drop":case"focusin":case"focusout":case"input":case"invalid":case"keydown":case"keypress":case"keyup":case"mousedown":case"mouseup":case"paste":case"pause":case"play":case"pointercancel":case"pointerdown":case"pointerup":case"ratechange":case"reset":case"resize":case"seeked":case"submit":case"touchcancel":case"touchend":case"touchstart":case"volumechange":case"change":case"selectionchange":case"textInput":case"compositionstart":case"compositionend":case"compositionupdate":case"beforeblur":case"afterblur":case"beforeinput":case"blur":case"fullscreenchange":case"focus":case"hashchange":case"popstate":case"select":case"selectstart":return 1;case"drag":case"dragenter":case"dragexit":case"dragleave":case"dragover":case"mousemove":case"mouseout":case"mouseover":case"pointermove":case"pointerout":case"pointerover":case"scroll":case"toggle":case"touchmove":case"wheel":case"mouseenter":case"mouseleave":case"pointerenter":case"pointerleave":return 4;case"message":switch(ot()){case st:return 1;case et:return 4;case St:case Bt:return 16;case It:return 536870912;default:return 16}default:return 16}}var Zi=null,Eu=null,_o=null;function $d(){if(_o)return _o;var n,i=Eu,a=i.length,l,d="value"in Zi?Zi.value:Zi.textContent,p=d.length;for(n=0;n<a&&i[n]===d[n];n++);var E=a-n;for(l=1;l<=E&&i[a-l]===d[p-l];l++);return _o=d.slice(n,1<l?1-l:void 0)}function vo(n){var i=n.keyCode;return"charCode"in n?(n=n.charCode,n===0&&i===13&&(n=13)):n=i,n===10&&(n=13),32<=n||n===13?n:0}function xo(){return!0}function Kd(){return!1}function In(n){function i(a,l,d,p,E){this._reactName=a,this._targetInst=d,this.type=l,this.nativeEvent=p,this.target=E,this.currentTarget=null;for(var D in n)n.hasOwnProperty(D)&&(a=n[D],this[D]=a?a(p):p[D]);return this.isDefaultPrevented=(p.defaultPrevented!=null?p.defaultPrevented:p.returnValue===!1)?xo:Kd,this.isPropagationStopped=Kd,this}return fe(i.prototype,{preventDefault:function(){this.defaultPrevented=!0;var a=this.nativeEvent;a&&(a.preventDefault?a.preventDefault():typeof a.returnValue!="unknown"&&(a.returnValue=!1),this.isDefaultPrevented=xo)},stopPropagation:function(){var a=this.nativeEvent;a&&(a.stopPropagation?a.stopPropagation():typeof a.cancelBubble!="unknown"&&(a.cancelBubble=!0),this.isPropagationStopped=xo)},persist:function(){},isPersistent:xo}),i}var as={eventPhase:0,bubbles:0,cancelable:0,timeStamp:function(n){return n.timeStamp||Date.now()},defaultPrevented:0,isTrusted:0},wu=In(as),xa=fe({},as,{view:0,detail:0}),T_=In(xa),Tu,Au,ya,yo=fe({},xa,{screenX:0,screenY:0,clientX:0,clientY:0,pageX:0,pageY:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,getModifierState:Ru,button:0,buttons:0,relatedTarget:function(n){return n.relatedTarget===void 0?n.fromElement===n.srcElement?n.toElement:n.fromElement:n.relatedTarget},movementX:function(n){return"movementX"in n?n.movementX:(n!==ya&&(ya&&n.type==="mousemove"?(Tu=n.screenX-ya.screenX,Au=n.screenY-ya.screenY):Au=Tu=0,ya=n),Tu)},movementY:function(n){return"movementY"in n?n.movementY:Au}}),Zd=In(yo),A_=fe({},yo,{dataTransfer:0}),C_=In(A_),R_=fe({},xa,{relatedTarget:0}),Cu=In(R_),b_=fe({},as,{animationName:0,elapsedTime:0,pseudoElement:0}),P_=In(b_),L_=fe({},as,{clipboardData:function(n){return"clipboardData"in n?n.clipboardData:window.clipboardData}}),N_=In(L_),D_=fe({},as,{data:0}),Qd=In(D_),I_={Esc:"Escape",Spacebar:" ",Left:"ArrowLeft",Up:"ArrowUp",Right:"ArrowRight",Down:"ArrowDown",Del:"Delete",Win:"OS",Menu:"ContextMenu",Apps:"ContextMenu",Scroll:"ScrollLock",MozPrintableKey:"Unidentified"},U_={8:"Backspace",9:"Tab",12:"Clear",13:"Enter",16:"Shift",17:"Control",18:"Alt",19:"Pause",20:"CapsLock",27:"Escape",32:" ",33:"PageUp",34:"PageDown",35:"End",36:"Home",37:"ArrowLeft",38:"ArrowUp",39:"ArrowRight",40:"ArrowDown",45:"Insert",46:"Delete",112:"F1",113:"F2",114:"F3",115:"F4",116:"F5",117:"F6",118:"F7",119:"F8",120:"F9",121:"F10",122:"F11",123:"F12",144:"NumLock",145:"ScrollLock",224:"Meta"},F_={Alt:"altKey",Control:"ctrlKey",Meta:"metaKey",Shift:"shiftKey"};function O_(n){var i=this.nativeEvent;return i.getModifierState?i.getModifierState(n):(n=F_[n])?!!i[n]:!1}function Ru(){return O_}var k_=fe({},xa,{key:function(n){if(n.key){var i=I_[n.key]||n.key;if(i!=="Unidentified")return i}return n.type==="keypress"?(n=vo(n),n===13?"Enter":String.fromCharCode(n)):n.type==="keydown"||n.type==="keyup"?U_[n.keyCode]||"Unidentified":""},code:0,location:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,repeat:0,locale:0,getModifierState:Ru,charCode:function(n){return n.type==="keypress"?vo(n):0},keyCode:function(n){return n.type==="keydown"||n.type==="keyup"?n.keyCode:0},which:function(n){return n.type==="keypress"?vo(n):n.type==="keydown"||n.type==="keyup"?n.keyCode:0}}),B_=In(k_),z_=fe({},yo,{pointerId:0,width:0,height:0,pressure:0,tangentialPressure:0,tiltX:0,tiltY:0,twist:0,pointerType:0,isPrimary:0}),Jd=In(z_),H_=fe({},xa,{touches:0,targetTouches:0,changedTouches:0,altKey:0,metaKey:0,ctrlKey:0,shiftKey:0,getModifierState:Ru}),V_=In(H_),G_=fe({},as,{propertyName:0,elapsedTime:0,pseudoElement:0}),W_=In(G_),j_=fe({},yo,{deltaX:function(n){return"deltaX"in n?n.deltaX:"wheelDeltaX"in n?-n.wheelDeltaX:0},deltaY:function(n){return"deltaY"in n?n.deltaY:"wheelDeltaY"in n?-n.wheelDeltaY:"wheelDelta"in n?-n.wheelDelta:0},deltaZ:0,deltaMode:0}),X_=In(j_),Y_=[9,13,27,32],bu=f&&"CompositionEvent"in window,Sa=null;f&&"documentMode"in document&&(Sa=document.documentMode);var q_=f&&"TextEvent"in window&&!Sa,eh=f&&(!bu||Sa&&8<Sa&&11>=Sa),th=" ",nh=!1;function ih(n,i){switch(n){case"keyup":return Y_.indexOf(i.keyCode)!==-1;case"keydown":return i.keyCode!==229;case"keypress":case"mousedown":case"focusout":return!0;default:return!1}}function rh(n){return n=n.detail,typeof n=="object"&&"data"in n?n.data:null}var os=!1;function $_(n,i){switch(n){case"compositionend":return rh(i);case"keypress":return i.which!==32?null:(nh=!0,th);case"textInput":return n=i.data,n===th&&nh?null:n;default:return null}}function K_(n,i){if(os)return n==="compositionend"||!bu&&ih(n,i)?(n=$d(),_o=Eu=Zi=null,os=!1,n):null;switch(n){case"paste":return null;case"keypress":if(!(i.ctrlKey||i.altKey||i.metaKey)||i.ctrlKey&&i.altKey){if(i.char&&1<i.char.length)return i.char;if(i.which)return String.fromCharCode(i.which)}return null;case"compositionend":return eh&&i.locale!=="ko"?null:i.data;default:return null}}var Z_={color:!0,date:!0,datetime:!0,"datetime-local":!0,email:!0,month:!0,number:!0,password:!0,range:!0,search:!0,tel:!0,text:!0,time:!0,url:!0,week:!0};function sh(n){var i=n&&n.nodeName&&n.nodeName.toLowerCase();return i==="input"?!!Z_[n.type]:i==="textarea"}function ah(n,i,a,l){vt(l),i=To(i,"onChange"),0<i.length&&(a=new wu("onChange","change",null,a,l),n.push({event:a,listeners:i}))}var Ma=null,Ea=null;function Q_(n){wh(n,0)}function So(n){var i=ds(n);if(B(i))return n}function J_(n,i){if(n==="change")return i}var oh=!1;if(f){var Pu;if(f){var Lu="oninput"in document;if(!Lu){var lh=document.createElement("div");lh.setAttribute("oninput","return;"),Lu=typeof lh.oninput=="function"}Pu=Lu}else Pu=!1;oh=Pu&&(!document.documentMode||9<document.documentMode)}function uh(){Ma&&(Ma.detachEvent("onpropertychange",ch),Ea=Ma=null)}function ch(n){if(n.propertyName==="value"&&So(Ea)){var i=[];ah(i,Ea,n,Ce(n)),qt(Q_,i)}}function ev(n,i,a){n==="focusin"?(uh(),Ma=i,Ea=a,Ma.attachEvent("onpropertychange",ch)):n==="focusout"&&uh()}function tv(n){if(n==="selectionchange"||n==="keyup"||n==="keydown")return So(Ea)}function nv(n,i){if(n==="click")return So(i)}function iv(n,i){if(n==="input"||n==="change")return So(i)}function rv(n,i){return n===i&&(n!==0||1/n===1/i)||n!==n&&i!==i}var Jn=typeof Object.is=="function"?Object.is:rv;function wa(n,i){if(Jn(n,i))return!0;if(typeof n!="object"||n===null||typeof i!="object"||i===null)return!1;var a=Object.keys(n),l=Object.keys(i);if(a.length!==l.length)return!1;for(l=0;l<a.length;l++){var d=a[l];if(!h.call(i,d)||!Jn(n[d],i[d]))return!1}return!0}function fh(n){for(;n&&n.firstChild;)n=n.firstChild;return n}function dh(n,i){var a=fh(n);n=0;for(var l;a;){if(a.nodeType===3){if(l=n+a.textContent.length,n<=i&&l>=i)return{node:a,offset:i-n};n=l}e:{for(;a;){if(a.nextSibling){a=a.nextSibling;break e}a=a.parentNode}a=void 0}a=fh(a)}}function hh(n,i){return n&&i?n===i?!0:n&&n.nodeType===3?!1:i&&i.nodeType===3?hh(n,i.parentNode):"contains"in n?n.contains(i):n.compareDocumentPosition?!!(n.compareDocumentPosition(i)&16):!1:!1}function ph(){for(var n=window,i=wt();i instanceof n.HTMLIFrameElement;){try{var a=typeof i.contentWindow.location.href=="string"}catch{a=!1}if(a)n=i.contentWindow;else break;i=wt(n.document)}return i}function Nu(n){var i=n&&n.nodeName&&n.nodeName.toLowerCase();return i&&(i==="input"&&(n.type==="text"||n.type==="search"||n.type==="tel"||n.type==="url"||n.type==="password")||i==="textarea"||n.contentEditable==="true")}function sv(n){var i=ph(),a=n.focusedElem,l=n.selectionRange;if(i!==a&&a&&a.ownerDocument&&hh(a.ownerDocument.documentElement,a)){if(l!==null&&Nu(a)){if(i=l.start,n=l.end,n===void 0&&(n=i),"selectionStart"in a)a.selectionStart=i,a.selectionEnd=Math.min(n,a.value.length);else if(n=(i=a.ownerDocument||document)&&i.defaultView||window,n.getSelection){n=n.getSelection();var d=a.textContent.length,p=Math.min(l.start,d);l=l.end===void 0?p:Math.min(l.end,d),!n.extend&&p>l&&(d=l,l=p,p=d),d=dh(a,p);var E=dh(a,l);d&&E&&(n.rangeCount!==1||n.anchorNode!==d.node||n.anchorOffset!==d.offset||n.focusNode!==E.node||n.focusOffset!==E.offset)&&(i=i.createRange(),i.setStart(d.node,d.offset),n.removeAllRanges(),p>l?(n.addRange(i),n.extend(E.node,E.offset)):(i.setEnd(E.node,E.offset),n.addRange(i)))}}for(i=[],n=a;n=n.parentNode;)n.nodeType===1&&i.push({element:n,left:n.scrollLeft,top:n.scrollTop});for(typeof a.focus=="function"&&a.focus(),a=0;a<i.length;a++)n=i[a],n.element.scrollLeft=n.left,n.element.scrollTop=n.top}}var av=f&&"documentMode"in document&&11>=document.documentMode,ls=null,Du=null,Ta=null,Iu=!1;function mh(n,i,a){var l=a.window===a?a.document:a.nodeType===9?a:a.ownerDocument;Iu||ls==null||ls!==wt(l)||(l=ls,"selectionStart"in l&&Nu(l)?l={start:l.selectionStart,end:l.selectionEnd}:(l=(l.ownerDocument&&l.ownerDocument.defaultView||window).getSelection(),l={anchorNode:l.anchorNode,anchorOffset:l.anchorOffset,focusNode:l.focusNode,focusOffset:l.focusOffset}),Ta&&wa(Ta,l)||(Ta=l,l=To(Du,"onSelect"),0<l.length&&(i=new wu("onSelect","select",null,i,a),n.push({event:i,listeners:l}),i.target=ls)))}function Mo(n,i){var a={};return a[n.toLowerCase()]=i.toLowerCase(),a["Webkit"+n]="webkit"+i,a["Moz"+n]="moz"+i,a}var us={animationend:Mo("Animation","AnimationEnd"),animationiteration:Mo("Animation","AnimationIteration"),animationstart:Mo("Animation","AnimationStart"),transitionend:Mo("Transition","TransitionEnd")},Uu={},gh={};f&&(gh=document.createElement("div").style,"AnimationEvent"in window||(delete us.animationend.animation,delete us.animationiteration.animation,delete us.animationstart.animation),"TransitionEvent"in window||delete us.transitionend.transition);function Eo(n){if(Uu[n])return Uu[n];if(!us[n])return n;var i=us[n],a;for(a in i)if(i.hasOwnProperty(a)&&a in gh)return Uu[n]=i[a];return n}var _h=Eo("animationend"),vh=Eo("animationiteration"),xh=Eo("animationstart"),yh=Eo("transitionend"),Sh=new Map,Mh="abort auxClick cancel canPlay canPlayThrough click close contextMenu copy cut drag dragEnd dragEnter dragExit dragLeave dragOver dragStart drop durationChange emptied encrypted ended error gotPointerCapture input invalid keyDown keyPress keyUp load loadedData loadedMetadata loadStart lostPointerCapture mouseDown mouseMove mouseOut mouseOver mouseUp paste pause play playing pointerCancel pointerDown pointerMove pointerOut pointerOver pointerUp progress rateChange reset resize seeked seeking stalled submit suspend timeUpdate touchCancel touchEnd touchStart volumeChange scroll toggle touchMove waiting wheel".split(" ");function Qi(n,i){Sh.set(n,i),u(i,[n])}for(var Fu=0;Fu<Mh.length;Fu++){var Ou=Mh[Fu],ov=Ou.toLowerCase(),lv=Ou[0].toUpperCase()+Ou.slice(1);Qi(ov,"on"+lv)}Qi(_h,"onAnimationEnd"),Qi(vh,"onAnimationIteration"),Qi(xh,"onAnimationStart"),Qi("dblclick","onDoubleClick"),Qi("focusin","onFocus"),Qi("focusout","onBlur"),Qi(yh,"onTransitionEnd"),c("onMouseEnter",["mouseout","mouseover"]),c("onMouseLeave",["mouseout","mouseover"]),c("onPointerEnter",["pointerout","pointerover"]),c("onPointerLeave",["pointerout","pointerover"]),u("onChange","change click focusin focusout input keydown keyup selectionchange".split(" ")),u("onSelect","focusout contextmenu dragend focusin keydown keyup mousedown mouseup selectionchange".split(" ")),u("onBeforeInput",["compositionend","keypress","textInput","paste"]),u("onCompositionEnd","compositionend focusout keydown keypress keyup mousedown".split(" ")),u("onCompositionStart","compositionstart focusout keydown keypress keyup mousedown".split(" ")),u("onCompositionUpdate","compositionupdate focusout keydown keypress keyup mousedown".split(" "));var Aa="abort canplay canplaythrough durationchange emptied encrypted ended error loadeddata loadedmetadata loadstart pause play playing progress ratechange resize seeked seeking stalled suspend timeupdate volumechange waiting".split(" "),uv=new Set("cancel close invalid load scroll toggle".split(" ").concat(Aa));function Eh(n,i,a){var l=n.type||"unknown-event";n.currentTarget=a,_u(l,i,void 0,n),n.currentTarget=null}function wh(n,i){i=(i&4)!==0;for(var a=0;a<n.length;a++){var l=n[a],d=l.event;l=l.listeners;e:{var p=void 0;if(i)for(var E=l.length-1;0<=E;E--){var D=l[E],z=D.instance,ae=D.currentTarget;if(D=D.listener,z!==p&&d.isPropagationStopped())break e;Eh(d,D,ae),p=z}else for(E=0;E<l.length;E++){if(D=l[E],z=D.instance,ae=D.currentTarget,D=D.listener,z!==p&&d.isPropagationStopped())break e;Eh(d,D,ae),p=z}}}if(Tr)throw n=Wi,Tr=!1,Wi=null,n}function Ut(n,i){var a=i[ju];a===void 0&&(a=i[ju]=new Set);var l=n+"__bubble";a.has(l)||(Th(i,n,2,!1),a.add(l))}function ku(n,i,a){var l=0;i&&(l|=4),Th(a,n,l,i)}var wo="_reactListening"+Math.random().toString(36).slice(2);function Ca(n){if(!n[wo]){n[wo]=!0,r.forEach(function(a){a!=="selectionchange"&&(uv.has(a)||ku(a,!1,n),ku(a,!0,n))});var i=n.nodeType===9?n:n.ownerDocument;i===null||i[wo]||(i[wo]=!0,ku("selectionchange",!1,i))}}function Th(n,i,a,l){switch(qd(i)){case 1:var d=E_;break;case 4:d=w_;break;default:d=Su}a=d.bind(null,i,a,n),d=void 0,!Qn||i!=="touchstart"&&i!=="touchmove"&&i!=="wheel"||(d=!0),l?d!==void 0?n.addEventListener(i,a,{capture:!0,passive:d}):n.addEventListener(i,a,!0):d!==void 0?n.addEventListener(i,a,{passive:d}):n.addEventListener(i,a,!1)}function Bu(n,i,a,l,d){var p=l;if((i&1)===0&&(i&2)===0&&l!==null)e:for(;;){if(l===null)return;var E=l.tag;if(E===3||E===4){var D=l.stateNode.containerInfo;if(D===d||D.nodeType===8&&D.parentNode===d)break;if(E===4)for(E=l.return;E!==null;){var z=E.tag;if((z===3||z===4)&&(z=E.stateNode.containerInfo,z===d||z.nodeType===8&&z.parentNode===d))return;E=E.return}for(;D!==null;){if(E=Cr(D),E===null)return;if(z=E.tag,z===5||z===6){l=p=E;continue e}D=D.parentNode}}l=l.return}qt(function(){var ae=p,ye=Ce(a),we=[];e:{var _e=Sh.get(n);if(_e!==void 0){var He=wu,je=n;switch(n){case"keypress":if(vo(a)===0)break e;case"keydown":case"keyup":He=B_;break;case"focusin":je="focus",He=Cu;break;case"focusout":je="blur",He=Cu;break;case"beforeblur":case"afterblur":He=Cu;break;case"click":if(a.button===2)break e;case"auxclick":case"dblclick":case"mousedown":case"mousemove":case"mouseup":case"mouseout":case"mouseover":case"contextmenu":He=Zd;break;case"drag":case"dragend":case"dragenter":case"dragexit":case"dragleave":case"dragover":case"dragstart":case"drop":He=C_;break;case"touchcancel":case"touchend":case"touchmove":case"touchstart":He=V_;break;case _h:case vh:case xh:He=P_;break;case yh:He=W_;break;case"scroll":He=T_;break;case"wheel":He=X_;break;case"copy":case"cut":case"paste":He=N_;break;case"gotpointercapture":case"lostpointercapture":case"pointercancel":case"pointerdown":case"pointermove":case"pointerout":case"pointerover":case"pointerup":He=Jd}var Ye=(i&4)!==0,jt=!Ye&&n==="scroll",$=Ye?_e!==null?_e+"Capture":null:_e;Ye=[];for(var W=ae,ee;W!==null;){ee=W;var be=ee.stateNode;if(ee.tag===5&&be!==null&&(ee=be,$!==null&&(be=En(W,$),be!=null&&Ye.push(Ra(W,be,ee)))),jt)break;W=W.return}0<Ye.length&&(_e=new He(_e,je,null,a,ye),we.push({event:_e,listeners:Ye}))}}if((i&7)===0){e:{if(_e=n==="mouseover"||n==="pointerover",He=n==="mouseout"||n==="pointerout",_e&&a!==j&&(je=a.relatedTarget||a.fromElement)&&(Cr(je)||je[Ti]))break e;if((He||_e)&&(_e=ye.window===ye?ye:(_e=ye.ownerDocument)?_e.defaultView||_e.parentWindow:window,He?(je=a.relatedTarget||a.toElement,He=ae,je=je?Cr(je):null,je!==null&&(jt=wi(je),je!==jt||je.tag!==5&&je.tag!==6)&&(je=null)):(He=null,je=ae),He!==je)){if(Ye=Zd,be="onMouseLeave",$="onMouseEnter",W="mouse",(n==="pointerout"||n==="pointerover")&&(Ye=Jd,be="onPointerLeave",$="onPointerEnter",W="pointer"),jt=He==null?_e:ds(He),ee=je==null?_e:ds(je),_e=new Ye(be,W+"leave",He,a,ye),_e.target=jt,_e.relatedTarget=ee,be=null,Cr(ye)===ae&&(Ye=new Ye($,W+"enter",je,a,ye),Ye.target=ee,Ye.relatedTarget=jt,be=Ye),jt=be,He&&je)t:{for(Ye=He,$=je,W=0,ee=Ye;ee;ee=cs(ee))W++;for(ee=0,be=$;be;be=cs(be))ee++;for(;0<W-ee;)Ye=cs(Ye),W--;for(;0<ee-W;)$=cs($),ee--;for(;W--;){if(Ye===$||$!==null&&Ye===$.alternate)break t;Ye=cs(Ye),$=cs($)}Ye=null}else Ye=null;He!==null&&Ah(we,_e,He,Ye,!1),je!==null&&jt!==null&&Ah(we,jt,je,Ye,!0)}}e:{if(_e=ae?ds(ae):window,He=_e.nodeName&&_e.nodeName.toLowerCase(),He==="select"||He==="input"&&_e.type==="file")var qe=J_;else if(sh(_e))if(oh)qe=iv;else{qe=tv;var nt=ev}else(He=_e.nodeName)&&He.toLowerCase()==="input"&&(_e.type==="checkbox"||_e.type==="radio")&&(qe=nv);if(qe&&(qe=qe(n,ae))){ah(we,qe,a,ye);break e}nt&&nt(n,_e,ae),n==="focusout"&&(nt=_e._wrapperState)&&nt.controlled&&_e.type==="number"&&Ue(_e,"number",_e.value)}switch(nt=ae?ds(ae):window,n){case"focusin":(sh(nt)||nt.contentEditable==="true")&&(ls=nt,Du=ae,Ta=null);break;case"focusout":Ta=Du=ls=null;break;case"mousedown":Iu=!0;break;case"contextmenu":case"mouseup":case"dragend":Iu=!1,mh(we,a,ye);break;case"selectionchange":if(av)break;case"keydown":case"keyup":mh(we,a,ye)}var it;if(bu)e:{switch(n){case"compositionstart":var ut="onCompositionStart";break e;case"compositionend":ut="onCompositionEnd";break e;case"compositionupdate":ut="onCompositionUpdate";break e}ut=void 0}else os?ih(n,a)&&(ut="onCompositionEnd"):n==="keydown"&&a.keyCode===229&&(ut="onCompositionStart");ut&&(eh&&a.locale!=="ko"&&(os||ut!=="onCompositionStart"?ut==="onCompositionEnd"&&os&&(it=$d()):(Zi=ye,Eu="value"in Zi?Zi.value:Zi.textContent,os=!0)),nt=To(ae,ut),0<nt.length&&(ut=new Qd(ut,n,null,a,ye),we.push({event:ut,listeners:nt}),it?ut.data=it:(it=rh(a),it!==null&&(ut.data=it)))),(it=q_?$_(n,a):K_(n,a))&&(ae=To(ae,"onBeforeInput"),0<ae.length&&(ye=new Qd("onBeforeInput","beforeinput",null,a,ye),we.push({event:ye,listeners:ae}),ye.data=it))}wh(we,i)})}function Ra(n,i,a){return{instance:n,listener:i,currentTarget:a}}function To(n,i){for(var a=i+"Capture",l=[];n!==null;){var d=n,p=d.stateNode;d.tag===5&&p!==null&&(d=p,p=En(n,a),p!=null&&l.unshift(Ra(n,p,d)),p=En(n,i),p!=null&&l.push(Ra(n,p,d))),n=n.return}return l}function cs(n){if(n===null)return null;do n=n.return;while(n&&n.tag!==5);return n||null}function Ah(n,i,a,l,d){for(var p=i._reactName,E=[];a!==null&&a!==l;){var D=a,z=D.alternate,ae=D.stateNode;if(z!==null&&z===l)break;D.tag===5&&ae!==null&&(D=ae,d?(z=En(a,p),z!=null&&E.unshift(Ra(a,z,D))):d||(z=En(a,p),z!=null&&E.push(Ra(a,z,D)))),a=a.return}E.length!==0&&n.push({event:i,listeners:E})}var cv=/\r\n?/g,fv=/\u0000|\uFFFD/g;function Ch(n){return(typeof n=="string"?n:""+n).replace(cv,`
`).replace(fv,"")}function Ao(n,i,a){if(i=Ch(i),Ch(n)!==i&&a)throw Error(t(425))}function Co(){}var zu=null,Hu=null;function Vu(n,i){return n==="textarea"||n==="noscript"||typeof i.children=="string"||typeof i.children=="number"||typeof i.dangerouslySetInnerHTML=="object"&&i.dangerouslySetInnerHTML!==null&&i.dangerouslySetInnerHTML.__html!=null}var Gu=typeof setTimeout=="function"?setTimeout:void 0,dv=typeof clearTimeout=="function"?clearTimeout:void 0,Rh=typeof Promise=="function"?Promise:void 0,hv=typeof queueMicrotask=="function"?queueMicrotask:typeof Rh<"u"?function(n){return Rh.resolve(null).then(n).catch(pv)}:Gu;function pv(n){setTimeout(function(){throw n})}function Wu(n,i){var a=i,l=0;do{var d=a.nextSibling;if(n.removeChild(a),d&&d.nodeType===8)if(a=d.data,a==="/$"){if(l===0){n.removeChild(d),va(i);return}l--}else a!=="$"&&a!=="$?"&&a!=="$!"||l++;a=d}while(a);va(i)}function Ji(n){for(;n!=null;n=n.nextSibling){var i=n.nodeType;if(i===1||i===3)break;if(i===8){if(i=n.data,i==="$"||i==="$!"||i==="$?")break;if(i==="/$")return null}}return n}function bh(n){n=n.previousSibling;for(var i=0;n;){if(n.nodeType===8){var a=n.data;if(a==="$"||a==="$!"||a==="$?"){if(i===0)return n;i--}else a==="/$"&&i++}n=n.previousSibling}return null}var fs=Math.random().toString(36).slice(2),pi="__reactFiber$"+fs,ba="__reactProps$"+fs,Ti="__reactContainer$"+fs,ju="__reactEvents$"+fs,mv="__reactListeners$"+fs,gv="__reactHandles$"+fs;function Cr(n){var i=n[pi];if(i)return i;for(var a=n.parentNode;a;){if(i=a[Ti]||a[pi]){if(a=i.alternate,i.child!==null||a!==null&&a.child!==null)for(n=bh(n);n!==null;){if(a=n[pi])return a;n=bh(n)}return i}n=a,a=n.parentNode}return null}function Pa(n){return n=n[pi]||n[Ti],!n||n.tag!==5&&n.tag!==6&&n.tag!==13&&n.tag!==3?null:n}function ds(n){if(n.tag===5||n.tag===6)return n.stateNode;throw Error(t(33))}function Ro(n){return n[ba]||null}var Xu=[],hs=-1;function er(n){return{current:n}}function Ft(n){0>hs||(n.current=Xu[hs],Xu[hs]=null,hs--)}function Dt(n,i){hs++,Xu[hs]=n.current,n.current=i}var tr={},cn=er(tr),Tn=er(!1),Rr=tr;function ps(n,i){var a=n.type.contextTypes;if(!a)return tr;var l=n.stateNode;if(l&&l.__reactInternalMemoizedUnmaskedChildContext===i)return l.__reactInternalMemoizedMaskedChildContext;var d={},p;for(p in a)d[p]=i[p];return l&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=i,n.__reactInternalMemoizedMaskedChildContext=d),d}function An(n){return n=n.childContextTypes,n!=null}function bo(){Ft(Tn),Ft(cn)}function Ph(n,i,a){if(cn.current!==tr)throw Error(t(168));Dt(cn,i),Dt(Tn,a)}function Lh(n,i,a){var l=n.stateNode;if(i=i.childContextTypes,typeof l.getChildContext!="function")return a;l=l.getChildContext();for(var d in l)if(!(d in i))throw Error(t(108,Me(n)||"Unknown",d));return fe({},a,l)}function Po(n){return n=(n=n.stateNode)&&n.__reactInternalMemoizedMergedChildContext||tr,Rr=cn.current,Dt(cn,n),Dt(Tn,Tn.current),!0}function Nh(n,i,a){var l=n.stateNode;if(!l)throw Error(t(169));a?(n=Lh(n,i,Rr),l.__reactInternalMemoizedMergedChildContext=n,Ft(Tn),Ft(cn),Dt(cn,n)):Ft(Tn),Dt(Tn,a)}var Ai=null,Lo=!1,Yu=!1;function Dh(n){Ai===null?Ai=[n]:Ai.push(n)}function _v(n){Lo=!0,Dh(n)}function nr(){if(!Yu&&Ai!==null){Yu=!0;var n=0,i=Pt;try{var a=Ai;for(Pt=1;n<a.length;n++){var l=a[n];do l=l(!0);while(l!==null)}Ai=null,Lo=!1}catch(d){throw Ai!==null&&(Ai=Ai.slice(n+1)),q(st,nr),d}finally{Pt=i,Yu=!1}}return null}var ms=[],gs=0,No=null,Do=0,Vn=[],Gn=0,br=null,Ci=1,Ri="";function Pr(n,i){ms[gs++]=Do,ms[gs++]=No,No=n,Do=i}function Ih(n,i,a){Vn[Gn++]=Ci,Vn[Gn++]=Ri,Vn[Gn++]=br,br=n;var l=Ci;n=Ri;var d=32-bt(l)-1;l&=~(1<<d),a+=1;var p=32-bt(i)+d;if(30<p){var E=d-d%5;p=(l&(1<<E)-1).toString(32),l>>=E,d-=E,Ci=1<<32-bt(i)+d|a<<d|l,Ri=p+n}else Ci=1<<p|a<<d|l,Ri=n}function qu(n){n.return!==null&&(Pr(n,1),Ih(n,1,0))}function $u(n){for(;n===No;)No=ms[--gs],ms[gs]=null,Do=ms[--gs],ms[gs]=null;for(;n===br;)br=Vn[--Gn],Vn[Gn]=null,Ri=Vn[--Gn],Vn[Gn]=null,Ci=Vn[--Gn],Vn[Gn]=null}var Un=null,Fn=null,zt=!1,ei=null;function Uh(n,i){var a=Yn(5,null,null,0);a.elementType="DELETED",a.stateNode=i,a.return=n,i=n.deletions,i===null?(n.deletions=[a],n.flags|=16):i.push(a)}function Fh(n,i){switch(n.tag){case 5:var a=n.type;return i=i.nodeType!==1||a.toLowerCase()!==i.nodeName.toLowerCase()?null:i,i!==null?(n.stateNode=i,Un=n,Fn=Ji(i.firstChild),!0):!1;case 6:return i=n.pendingProps===""||i.nodeType!==3?null:i,i!==null?(n.stateNode=i,Un=n,Fn=null,!0):!1;case 13:return i=i.nodeType!==8?null:i,i!==null?(a=br!==null?{id:Ci,overflow:Ri}:null,n.memoizedState={dehydrated:i,treeContext:a,retryLane:1073741824},a=Yn(18,null,null,0),a.stateNode=i,a.return=n,n.child=a,Un=n,Fn=null,!0):!1;default:return!1}}function Ku(n){return(n.mode&1)!==0&&(n.flags&128)===0}function Zu(n){if(zt){var i=Fn;if(i){var a=i;if(!Fh(n,i)){if(Ku(n))throw Error(t(418));i=Ji(a.nextSibling);var l=Un;i&&Fh(n,i)?Uh(l,a):(n.flags=n.flags&-4097|2,zt=!1,Un=n)}}else{if(Ku(n))throw Error(t(418));n.flags=n.flags&-4097|2,zt=!1,Un=n}}}function Oh(n){for(n=n.return;n!==null&&n.tag!==5&&n.tag!==3&&n.tag!==13;)n=n.return;Un=n}function Io(n){if(n!==Un)return!1;if(!zt)return Oh(n),zt=!0,!1;var i;if((i=n.tag!==3)&&!(i=n.tag!==5)&&(i=n.type,i=i!=="head"&&i!=="body"&&!Vu(n.type,n.memoizedProps)),i&&(i=Fn)){if(Ku(n))throw kh(),Error(t(418));for(;i;)Uh(n,i),i=Ji(i.nextSibling)}if(Oh(n),n.tag===13){if(n=n.memoizedState,n=n!==null?n.dehydrated:null,!n)throw Error(t(317));e:{for(n=n.nextSibling,i=0;n;){if(n.nodeType===8){var a=n.data;if(a==="/$"){if(i===0){Fn=Ji(n.nextSibling);break e}i--}else a!=="$"&&a!=="$!"&&a!=="$?"||i++}n=n.nextSibling}Fn=null}}else Fn=Un?Ji(n.stateNode.nextSibling):null;return!0}function kh(){for(var n=Fn;n;)n=Ji(n.nextSibling)}function _s(){Fn=Un=null,zt=!1}function Qu(n){ei===null?ei=[n]:ei.push(n)}var vv=L.ReactCurrentBatchConfig;function La(n,i,a){if(n=a.ref,n!==null&&typeof n!="function"&&typeof n!="object"){if(a._owner){if(a=a._owner,a){if(a.tag!==1)throw Error(t(309));var l=a.stateNode}if(!l)throw Error(t(147,n));var d=l,p=""+n;return i!==null&&i.ref!==null&&typeof i.ref=="function"&&i.ref._stringRef===p?i.ref:(i=function(E){var D=d.refs;E===null?delete D[p]:D[p]=E},i._stringRef=p,i)}if(typeof n!="string")throw Error(t(284));if(!a._owner)throw Error(t(290,n))}return n}function Uo(n,i){throw n=Object.prototype.toString.call(i),Error(t(31,n==="[object Object]"?"object with keys {"+Object.keys(i).join(", ")+"}":n))}function Bh(n){var i=n._init;return i(n._payload)}function zh(n){function i($,W){if(n){var ee=$.deletions;ee===null?($.deletions=[W],$.flags|=16):ee.push(W)}}function a($,W){if(!n)return null;for(;W!==null;)i($,W),W=W.sibling;return null}function l($,W){for($=new Map;W!==null;)W.key!==null?$.set(W.key,W):$.set(W.index,W),W=W.sibling;return $}function d($,W){return $=cr($,W),$.index=0,$.sibling=null,$}function p($,W,ee){return $.index=ee,n?(ee=$.alternate,ee!==null?(ee=ee.index,ee<W?($.flags|=2,W):ee):($.flags|=2,W)):($.flags|=1048576,W)}function E($){return n&&$.alternate===null&&($.flags|=2),$}function D($,W,ee,be){return W===null||W.tag!==6?(W=Gc(ee,$.mode,be),W.return=$,W):(W=d(W,ee),W.return=$,W)}function z($,W,ee,be){var qe=ee.type;return qe===N?ye($,W,ee.props.children,be,ee.key):W!==null&&(W.elementType===qe||typeof qe=="object"&&qe!==null&&qe.$$typeof===ue&&Bh(qe)===W.type)?(be=d(W,ee.props),be.ref=La($,W,ee),be.return=$,be):(be=sl(ee.type,ee.key,ee.props,null,$.mode,be),be.ref=La($,W,ee),be.return=$,be)}function ae($,W,ee,be){return W===null||W.tag!==4||W.stateNode.containerInfo!==ee.containerInfo||W.stateNode.implementation!==ee.implementation?(W=Wc(ee,$.mode,be),W.return=$,W):(W=d(W,ee.children||[]),W.return=$,W)}function ye($,W,ee,be,qe){return W===null||W.tag!==7?(W=kr(ee,$.mode,be,qe),W.return=$,W):(W=d(W,ee),W.return=$,W)}function we($,W,ee){if(typeof W=="string"&&W!==""||typeof W=="number")return W=Gc(""+W,$.mode,ee),W.return=$,W;if(typeof W=="object"&&W!==null){switch(W.$$typeof){case Y:return ee=sl(W.type,W.key,W.props,null,$.mode,ee),ee.ref=La($,null,W),ee.return=$,ee;case U:return W=Wc(W,$.mode,ee),W.return=$,W;case ue:var be=W._init;return we($,be(W._payload),ee)}if(F(W)||he(W))return W=kr(W,$.mode,ee,null),W.return=$,W;Uo($,W)}return null}function _e($,W,ee,be){var qe=W!==null?W.key:null;if(typeof ee=="string"&&ee!==""||typeof ee=="number")return qe!==null?null:D($,W,""+ee,be);if(typeof ee=="object"&&ee!==null){switch(ee.$$typeof){case Y:return ee.key===qe?z($,W,ee,be):null;case U:return ee.key===qe?ae($,W,ee,be):null;case ue:return qe=ee._init,_e($,W,qe(ee._payload),be)}if(F(ee)||he(ee))return qe!==null?null:ye($,W,ee,be,null);Uo($,ee)}return null}function He($,W,ee,be,qe){if(typeof be=="string"&&be!==""||typeof be=="number")return $=$.get(ee)||null,D(W,$,""+be,qe);if(typeof be=="object"&&be!==null){switch(be.$$typeof){case Y:return $=$.get(be.key===null?ee:be.key)||null,z(W,$,be,qe);case U:return $=$.get(be.key===null?ee:be.key)||null,ae(W,$,be,qe);case ue:var nt=be._init;return He($,W,ee,nt(be._payload),qe)}if(F(be)||he(be))return $=$.get(ee)||null,ye(W,$,be,qe,null);Uo(W,be)}return null}function je($,W,ee,be){for(var qe=null,nt=null,it=W,ut=W=0,an=null;it!==null&&ut<ee.length;ut++){it.index>ut?(an=it,it=null):an=it.sibling;var At=_e($,it,ee[ut],be);if(At===null){it===null&&(it=an);break}n&&it&&At.alternate===null&&i($,it),W=p(At,W,ut),nt===null?qe=At:nt.sibling=At,nt=At,it=an}if(ut===ee.length)return a($,it),zt&&Pr($,ut),qe;if(it===null){for(;ut<ee.length;ut++)it=we($,ee[ut],be),it!==null&&(W=p(it,W,ut),nt===null?qe=it:nt.sibling=it,nt=it);return zt&&Pr($,ut),qe}for(it=l($,it);ut<ee.length;ut++)an=He(it,$,ut,ee[ut],be),an!==null&&(n&&an.alternate!==null&&it.delete(an.key===null?ut:an.key),W=p(an,W,ut),nt===null?qe=an:nt.sibling=an,nt=an);return n&&it.forEach(function(fr){return i($,fr)}),zt&&Pr($,ut),qe}function Ye($,W,ee,be){var qe=he(ee);if(typeof qe!="function")throw Error(t(150));if(ee=qe.call(ee),ee==null)throw Error(t(151));for(var nt=qe=null,it=W,ut=W=0,an=null,At=ee.next();it!==null&&!At.done;ut++,At=ee.next()){it.index>ut?(an=it,it=null):an=it.sibling;var fr=_e($,it,At.value,be);if(fr===null){it===null&&(it=an);break}n&&it&&fr.alternate===null&&i($,it),W=p(fr,W,ut),nt===null?qe=fr:nt.sibling=fr,nt=fr,it=an}if(At.done)return a($,it),zt&&Pr($,ut),qe;if(it===null){for(;!At.done;ut++,At=ee.next())At=we($,At.value,be),At!==null&&(W=p(At,W,ut),nt===null?qe=At:nt.sibling=At,nt=At);return zt&&Pr($,ut),qe}for(it=l($,it);!At.done;ut++,At=ee.next())At=He(it,$,ut,At.value,be),At!==null&&(n&&At.alternate!==null&&it.delete(At.key===null?ut:At.key),W=p(At,W,ut),nt===null?qe=At:nt.sibling=At,nt=At);return n&&it.forEach(function(Zv){return i($,Zv)}),zt&&Pr($,ut),qe}function jt($,W,ee,be){if(typeof ee=="object"&&ee!==null&&ee.type===N&&ee.key===null&&(ee=ee.props.children),typeof ee=="object"&&ee!==null){switch(ee.$$typeof){case Y:e:{for(var qe=ee.key,nt=W;nt!==null;){if(nt.key===qe){if(qe=ee.type,qe===N){if(nt.tag===7){a($,nt.sibling),W=d(nt,ee.props.children),W.return=$,$=W;break e}}else if(nt.elementType===qe||typeof qe=="object"&&qe!==null&&qe.$$typeof===ue&&Bh(qe)===nt.type){a($,nt.sibling),W=d(nt,ee.props),W.ref=La($,nt,ee),W.return=$,$=W;break e}a($,nt);break}else i($,nt);nt=nt.sibling}ee.type===N?(W=kr(ee.props.children,$.mode,be,ee.key),W.return=$,$=W):(be=sl(ee.type,ee.key,ee.props,null,$.mode,be),be.ref=La($,W,ee),be.return=$,$=be)}return E($);case U:e:{for(nt=ee.key;W!==null;){if(W.key===nt)if(W.tag===4&&W.stateNode.containerInfo===ee.containerInfo&&W.stateNode.implementation===ee.implementation){a($,W.sibling),W=d(W,ee.children||[]),W.return=$,$=W;break e}else{a($,W);break}else i($,W);W=W.sibling}W=Wc(ee,$.mode,be),W.return=$,$=W}return E($);case ue:return nt=ee._init,jt($,W,nt(ee._payload),be)}if(F(ee))return je($,W,ee,be);if(he(ee))return Ye($,W,ee,be);Uo($,ee)}return typeof ee=="string"&&ee!==""||typeof ee=="number"?(ee=""+ee,W!==null&&W.tag===6?(a($,W.sibling),W=d(W,ee),W.return=$,$=W):(a($,W),W=Gc(ee,$.mode,be),W.return=$,$=W),E($)):a($,W)}return jt}var vs=zh(!0),Hh=zh(!1),Fo=er(null),Oo=null,xs=null,Ju=null;function ec(){Ju=xs=Oo=null}function tc(n){var i=Fo.current;Ft(Fo),n._currentValue=i}function nc(n,i,a){for(;n!==null;){var l=n.alternate;if((n.childLanes&i)!==i?(n.childLanes|=i,l!==null&&(l.childLanes|=i)):l!==null&&(l.childLanes&i)!==i&&(l.childLanes|=i),n===a)break;n=n.return}}function ys(n,i){Oo=n,Ju=xs=null,n=n.dependencies,n!==null&&n.firstContext!==null&&((n.lanes&i)!==0&&(Cn=!0),n.firstContext=null)}function Wn(n){var i=n._currentValue;if(Ju!==n)if(n={context:n,memoizedValue:i,next:null},xs===null){if(Oo===null)throw Error(t(308));xs=n,Oo.dependencies={lanes:0,firstContext:n}}else xs=xs.next=n;return i}var Lr=null;function ic(n){Lr===null?Lr=[n]:Lr.push(n)}function Vh(n,i,a,l){var d=i.interleaved;return d===null?(a.next=a,ic(i)):(a.next=d.next,d.next=a),i.interleaved=a,bi(n,l)}function bi(n,i){n.lanes|=i;var a=n.alternate;for(a!==null&&(a.lanes|=i),a=n,n=n.return;n!==null;)n.childLanes|=i,a=n.alternate,a!==null&&(a.childLanes|=i),a=n,n=n.return;return a.tag===3?a.stateNode:null}var ir=!1;function rc(n){n.updateQueue={baseState:n.memoizedState,firstBaseUpdate:null,lastBaseUpdate:null,shared:{pending:null,interleaved:null,lanes:0},effects:null}}function Gh(n,i){n=n.updateQueue,i.updateQueue===n&&(i.updateQueue={baseState:n.baseState,firstBaseUpdate:n.firstBaseUpdate,lastBaseUpdate:n.lastBaseUpdate,shared:n.shared,effects:n.effects})}function Pi(n,i){return{eventTime:n,lane:i,tag:0,payload:null,callback:null,next:null}}function rr(n,i,a){var l=n.updateQueue;if(l===null)return null;if(l=l.shared,(Mt&2)!==0){var d=l.pending;return d===null?i.next=i:(i.next=d.next,d.next=i),l.pending=i,bi(n,a)}return d=l.interleaved,d===null?(i.next=i,ic(l)):(i.next=d.next,d.next=i),l.interleaved=i,bi(n,a)}function ko(n,i,a){if(i=i.updateQueue,i!==null&&(i=i.shared,(a&4194240)!==0)){var l=i.lanes;l&=n.pendingLanes,a|=l,i.lanes=a,vu(n,a)}}function Wh(n,i){var a=n.updateQueue,l=n.alternate;if(l!==null&&(l=l.updateQueue,a===l)){var d=null,p=null;if(a=a.firstBaseUpdate,a!==null){do{var E={eventTime:a.eventTime,lane:a.lane,tag:a.tag,payload:a.payload,callback:a.callback,next:null};p===null?d=p=E:p=p.next=E,a=a.next}while(a!==null);p===null?d=p=i:p=p.next=i}else d=p=i;a={baseState:l.baseState,firstBaseUpdate:d,lastBaseUpdate:p,shared:l.shared,effects:l.effects},n.updateQueue=a;return}n=a.lastBaseUpdate,n===null?a.firstBaseUpdate=i:n.next=i,a.lastBaseUpdate=i}function Bo(n,i,a,l){var d=n.updateQueue;ir=!1;var p=d.firstBaseUpdate,E=d.lastBaseUpdate,D=d.shared.pending;if(D!==null){d.shared.pending=null;var z=D,ae=z.next;z.next=null,E===null?p=ae:E.next=ae,E=z;var ye=n.alternate;ye!==null&&(ye=ye.updateQueue,D=ye.lastBaseUpdate,D!==E&&(D===null?ye.firstBaseUpdate=ae:D.next=ae,ye.lastBaseUpdate=z))}if(p!==null){var we=d.baseState;E=0,ye=ae=z=null,D=p;do{var _e=D.lane,He=D.eventTime;if((l&_e)===_e){ye!==null&&(ye=ye.next={eventTime:He,lane:0,tag:D.tag,payload:D.payload,callback:D.callback,next:null});e:{var je=n,Ye=D;switch(_e=i,He=a,Ye.tag){case 1:if(je=Ye.payload,typeof je=="function"){we=je.call(He,we,_e);break e}we=je;break e;case 3:je.flags=je.flags&-65537|128;case 0:if(je=Ye.payload,_e=typeof je=="function"?je.call(He,we,_e):je,_e==null)break e;we=fe({},we,_e);break e;case 2:ir=!0}}D.callback!==null&&D.lane!==0&&(n.flags|=64,_e=d.effects,_e===null?d.effects=[D]:_e.push(D))}else He={eventTime:He,lane:_e,tag:D.tag,payload:D.payload,callback:D.callback,next:null},ye===null?(ae=ye=He,z=we):ye=ye.next=He,E|=_e;if(D=D.next,D===null){if(D=d.shared.pending,D===null)break;_e=D,D=_e.next,_e.next=null,d.lastBaseUpdate=_e,d.shared.pending=null}}while(!0);if(ye===null&&(z=we),d.baseState=z,d.firstBaseUpdate=ae,d.lastBaseUpdate=ye,i=d.shared.interleaved,i!==null){d=i;do E|=d.lane,d=d.next;while(d!==i)}else p===null&&(d.shared.lanes=0);Ir|=E,n.lanes=E,n.memoizedState=we}}function jh(n,i,a){if(n=i.effects,i.effects=null,n!==null)for(i=0;i<n.length;i++){var l=n[i],d=l.callback;if(d!==null){if(l.callback=null,l=a,typeof d!="function")throw Error(t(191,d));d.call(l)}}}var Na={},mi=er(Na),Da=er(Na),Ia=er(Na);function Nr(n){if(n===Na)throw Error(t(174));return n}function sc(n,i){switch(Dt(Ia,i),Dt(Da,n),Dt(mi,Na),n=i.nodeType,n){case 9:case 11:i=(i=i.documentElement)?i.namespaceURI:Ne(null,"");break;default:n=n===8?i.parentNode:i,i=n.namespaceURI||null,n=n.tagName,i=Ne(i,n)}Ft(mi),Dt(mi,i)}function Ss(){Ft(mi),Ft(Da),Ft(Ia)}function Xh(n){Nr(Ia.current);var i=Nr(mi.current),a=Ne(i,n.type);i!==a&&(Dt(Da,n),Dt(mi,a))}function ac(n){Da.current===n&&(Ft(mi),Ft(Da))}var Ht=er(0);function zo(n){for(var i=n;i!==null;){if(i.tag===13){var a=i.memoizedState;if(a!==null&&(a=a.dehydrated,a===null||a.data==="$?"||a.data==="$!"))return i}else if(i.tag===19&&i.memoizedProps.revealOrder!==void 0){if((i.flags&128)!==0)return i}else if(i.child!==null){i.child.return=i,i=i.child;continue}if(i===n)break;for(;i.sibling===null;){if(i.return===null||i.return===n)return null;i=i.return}i.sibling.return=i.return,i=i.sibling}return null}var oc=[];function lc(){for(var n=0;n<oc.length;n++)oc[n]._workInProgressVersionPrimary=null;oc.length=0}var Ho=L.ReactCurrentDispatcher,uc=L.ReactCurrentBatchConfig,Dr=0,Vt=null,$t=null,rn=null,Vo=!1,Ua=!1,Fa=0,xv=0;function fn(){throw Error(t(321))}function cc(n,i){if(i===null)return!1;for(var a=0;a<i.length&&a<n.length;a++)if(!Jn(n[a],i[a]))return!1;return!0}function fc(n,i,a,l,d,p){if(Dr=p,Vt=i,i.memoizedState=null,i.updateQueue=null,i.lanes=0,Ho.current=n===null||n.memoizedState===null?Ev:wv,n=a(l,d),Ua){p=0;do{if(Ua=!1,Fa=0,25<=p)throw Error(t(301));p+=1,rn=$t=null,i.updateQueue=null,Ho.current=Tv,n=a(l,d)}while(Ua)}if(Ho.current=jo,i=$t!==null&&$t.next!==null,Dr=0,rn=$t=Vt=null,Vo=!1,i)throw Error(t(300));return n}function dc(){var n=Fa!==0;return Fa=0,n}function gi(){var n={memoizedState:null,baseState:null,baseQueue:null,queue:null,next:null};return rn===null?Vt.memoizedState=rn=n:rn=rn.next=n,rn}function jn(){if($t===null){var n=Vt.alternate;n=n!==null?n.memoizedState:null}else n=$t.next;var i=rn===null?Vt.memoizedState:rn.next;if(i!==null)rn=i,$t=n;else{if(n===null)throw Error(t(310));$t=n,n={memoizedState:$t.memoizedState,baseState:$t.baseState,baseQueue:$t.baseQueue,queue:$t.queue,next:null},rn===null?Vt.memoizedState=rn=n:rn=rn.next=n}return rn}function Oa(n,i){return typeof i=="function"?i(n):i}function hc(n){var i=jn(),a=i.queue;if(a===null)throw Error(t(311));a.lastRenderedReducer=n;var l=$t,d=l.baseQueue,p=a.pending;if(p!==null){if(d!==null){var E=d.next;d.next=p.next,p.next=E}l.baseQueue=d=p,a.pending=null}if(d!==null){p=d.next,l=l.baseState;var D=E=null,z=null,ae=p;do{var ye=ae.lane;if((Dr&ye)===ye)z!==null&&(z=z.next={lane:0,action:ae.action,hasEagerState:ae.hasEagerState,eagerState:ae.eagerState,next:null}),l=ae.hasEagerState?ae.eagerState:n(l,ae.action);else{var we={lane:ye,action:ae.action,hasEagerState:ae.hasEagerState,eagerState:ae.eagerState,next:null};z===null?(D=z=we,E=l):z=z.next=we,Vt.lanes|=ye,Ir|=ye}ae=ae.next}while(ae!==null&&ae!==p);z===null?E=l:z.next=D,Jn(l,i.memoizedState)||(Cn=!0),i.memoizedState=l,i.baseState=E,i.baseQueue=z,a.lastRenderedState=l}if(n=a.interleaved,n!==null){d=n;do p=d.lane,Vt.lanes|=p,Ir|=p,d=d.next;while(d!==n)}else d===null&&(a.lanes=0);return[i.memoizedState,a.dispatch]}function pc(n){var i=jn(),a=i.queue;if(a===null)throw Error(t(311));a.lastRenderedReducer=n;var l=a.dispatch,d=a.pending,p=i.memoizedState;if(d!==null){a.pending=null;var E=d=d.next;do p=n(p,E.action),E=E.next;while(E!==d);Jn(p,i.memoizedState)||(Cn=!0),i.memoizedState=p,i.baseQueue===null&&(i.baseState=p),a.lastRenderedState=p}return[p,l]}function Yh(){}function qh(n,i){var a=Vt,l=jn(),d=i(),p=!Jn(l.memoizedState,d);if(p&&(l.memoizedState=d,Cn=!0),l=l.queue,mc(Zh.bind(null,a,l,n),[n]),l.getSnapshot!==i||p||rn!==null&&rn.memoizedState.tag&1){if(a.flags|=2048,ka(9,Kh.bind(null,a,l,d,i),void 0,null),sn===null)throw Error(t(349));(Dr&30)!==0||$h(a,i,d)}return d}function $h(n,i,a){n.flags|=16384,n={getSnapshot:i,value:a},i=Vt.updateQueue,i===null?(i={lastEffect:null,stores:null},Vt.updateQueue=i,i.stores=[n]):(a=i.stores,a===null?i.stores=[n]:a.push(n))}function Kh(n,i,a,l){i.value=a,i.getSnapshot=l,Qh(i)&&Jh(n)}function Zh(n,i,a){return a(function(){Qh(i)&&Jh(n)})}function Qh(n){var i=n.getSnapshot;n=n.value;try{var a=i();return!Jn(n,a)}catch{return!0}}function Jh(n){var i=bi(n,1);i!==null&&ri(i,n,1,-1)}function ep(n){var i=gi();return typeof n=="function"&&(n=n()),i.memoizedState=i.baseState=n,n={pending:null,interleaved:null,lanes:0,dispatch:null,lastRenderedReducer:Oa,lastRenderedState:n},i.queue=n,n=n.dispatch=Mv.bind(null,Vt,n),[i.memoizedState,n]}function ka(n,i,a,l){return n={tag:n,create:i,destroy:a,deps:l,next:null},i=Vt.updateQueue,i===null?(i={lastEffect:null,stores:null},Vt.updateQueue=i,i.lastEffect=n.next=n):(a=i.lastEffect,a===null?i.lastEffect=n.next=n:(l=a.next,a.next=n,n.next=l,i.lastEffect=n)),n}function tp(){return jn().memoizedState}function Go(n,i,a,l){var d=gi();Vt.flags|=n,d.memoizedState=ka(1|i,a,void 0,l===void 0?null:l)}function Wo(n,i,a,l){var d=jn();l=l===void 0?null:l;var p=void 0;if($t!==null){var E=$t.memoizedState;if(p=E.destroy,l!==null&&cc(l,E.deps)){d.memoizedState=ka(i,a,p,l);return}}Vt.flags|=n,d.memoizedState=ka(1|i,a,p,l)}function np(n,i){return Go(8390656,8,n,i)}function mc(n,i){return Wo(2048,8,n,i)}function ip(n,i){return Wo(4,2,n,i)}function rp(n,i){return Wo(4,4,n,i)}function sp(n,i){if(typeof i=="function")return n=n(),i(n),function(){i(null)};if(i!=null)return n=n(),i.current=n,function(){i.current=null}}function ap(n,i,a){return a=a!=null?a.concat([n]):null,Wo(4,4,sp.bind(null,i,n),a)}function gc(){}function op(n,i){var a=jn();i=i===void 0?null:i;var l=a.memoizedState;return l!==null&&i!==null&&cc(i,l[1])?l[0]:(a.memoizedState=[n,i],n)}function lp(n,i){var a=jn();i=i===void 0?null:i;var l=a.memoizedState;return l!==null&&i!==null&&cc(i,l[1])?l[0]:(n=n(),a.memoizedState=[n,i],n)}function up(n,i,a){return(Dr&21)===0?(n.baseState&&(n.baseState=!1,Cn=!0),n.memoizedState=a):(Jn(a,i)||(a=fo(),Vt.lanes|=a,Ir|=a,n.baseState=!0),i)}function yv(n,i){var a=Pt;Pt=a!==0&&4>a?a:4,n(!0);var l=uc.transition;uc.transition={};try{n(!1),i()}finally{Pt=a,uc.transition=l}}function cp(){return jn().memoizedState}function Sv(n,i,a){var l=lr(n);if(a={lane:l,action:a,hasEagerState:!1,eagerState:null,next:null},fp(n))dp(i,a);else if(a=Vh(n,i,a,l),a!==null){var d=vn();ri(a,n,l,d),hp(a,i,l)}}function Mv(n,i,a){var l=lr(n),d={lane:l,action:a,hasEagerState:!1,eagerState:null,next:null};if(fp(n))dp(i,d);else{var p=n.alternate;if(n.lanes===0&&(p===null||p.lanes===0)&&(p=i.lastRenderedReducer,p!==null))try{var E=i.lastRenderedState,D=p(E,a);if(d.hasEagerState=!0,d.eagerState=D,Jn(D,E)){var z=i.interleaved;z===null?(d.next=d,ic(i)):(d.next=z.next,z.next=d),i.interleaved=d;return}}catch{}finally{}a=Vh(n,i,d,l),a!==null&&(d=vn(),ri(a,n,l,d),hp(a,i,l))}}function fp(n){var i=n.alternate;return n===Vt||i!==null&&i===Vt}function dp(n,i){Ua=Vo=!0;var a=n.pending;a===null?i.next=i:(i.next=a.next,a.next=i),n.pending=i}function hp(n,i,a){if((a&4194240)!==0){var l=i.lanes;l&=n.pendingLanes,a|=l,i.lanes=a,vu(n,a)}}var jo={readContext:Wn,useCallback:fn,useContext:fn,useEffect:fn,useImperativeHandle:fn,useInsertionEffect:fn,useLayoutEffect:fn,useMemo:fn,useReducer:fn,useRef:fn,useState:fn,useDebugValue:fn,useDeferredValue:fn,useTransition:fn,useMutableSource:fn,useSyncExternalStore:fn,useId:fn,unstable_isNewReconciler:!1},Ev={readContext:Wn,useCallback:function(n,i){return gi().memoizedState=[n,i===void 0?null:i],n},useContext:Wn,useEffect:np,useImperativeHandle:function(n,i,a){return a=a!=null?a.concat([n]):null,Go(4194308,4,sp.bind(null,i,n),a)},useLayoutEffect:function(n,i){return Go(4194308,4,n,i)},useInsertionEffect:function(n,i){return Go(4,2,n,i)},useMemo:function(n,i){var a=gi();return i=i===void 0?null:i,n=n(),a.memoizedState=[n,i],n},useReducer:function(n,i,a){var l=gi();return i=a!==void 0?a(i):i,l.memoizedState=l.baseState=i,n={pending:null,interleaved:null,lanes:0,dispatch:null,lastRenderedReducer:n,lastRenderedState:i},l.queue=n,n=n.dispatch=Sv.bind(null,Vt,n),[l.memoizedState,n]},useRef:function(n){var i=gi();return n={current:n},i.memoizedState=n},useState:ep,useDebugValue:gc,useDeferredValue:function(n){return gi().memoizedState=n},useTransition:function(){var n=ep(!1),i=n[0];return n=yv.bind(null,n[1]),gi().memoizedState=n,[i,n]},useMutableSource:function(){},useSyncExternalStore:function(n,i,a){var l=Vt,d=gi();if(zt){if(a===void 0)throw Error(t(407));a=a()}else{if(a=i(),sn===null)throw Error(t(349));(Dr&30)!==0||$h(l,i,a)}d.memoizedState=a;var p={value:a,getSnapshot:i};return d.queue=p,np(Zh.bind(null,l,p,n),[n]),l.flags|=2048,ka(9,Kh.bind(null,l,p,a,i),void 0,null),a},useId:function(){var n=gi(),i=sn.identifierPrefix;if(zt){var a=Ri,l=Ci;a=(l&~(1<<32-bt(l)-1)).toString(32)+a,i=":"+i+"R"+a,a=Fa++,0<a&&(i+="H"+a.toString(32)),i+=":"}else a=xv++,i=":"+i+"r"+a.toString(32)+":";return n.memoizedState=i},unstable_isNewReconciler:!1},wv={readContext:Wn,useCallback:op,useContext:Wn,useEffect:mc,useImperativeHandle:ap,useInsertionEffect:ip,useLayoutEffect:rp,useMemo:lp,useReducer:hc,useRef:tp,useState:function(){return hc(Oa)},useDebugValue:gc,useDeferredValue:function(n){var i=jn();return up(i,$t.memoizedState,n)},useTransition:function(){var n=hc(Oa)[0],i=jn().memoizedState;return[n,i]},useMutableSource:Yh,useSyncExternalStore:qh,useId:cp,unstable_isNewReconciler:!1},Tv={readContext:Wn,useCallback:op,useContext:Wn,useEffect:mc,useImperativeHandle:ap,useInsertionEffect:ip,useLayoutEffect:rp,useMemo:lp,useReducer:pc,useRef:tp,useState:function(){return pc(Oa)},useDebugValue:gc,useDeferredValue:function(n){var i=jn();return $t===null?i.memoizedState=n:up(i,$t.memoizedState,n)},useTransition:function(){var n=pc(Oa)[0],i=jn().memoizedState;return[n,i]},useMutableSource:Yh,useSyncExternalStore:qh,useId:cp,unstable_isNewReconciler:!1};function ti(n,i){if(n&&n.defaultProps){i=fe({},i),n=n.defaultProps;for(var a in n)i[a]===void 0&&(i[a]=n[a]);return i}return i}function _c(n,i,a,l){i=n.memoizedState,a=a(l,i),a=a==null?i:fe({},i,a),n.memoizedState=a,n.lanes===0&&(n.updateQueue.baseState=a)}var Xo={isMounted:function(n){return(n=n._reactInternals)?wi(n)===n:!1},enqueueSetState:function(n,i,a){n=n._reactInternals;var l=vn(),d=lr(n),p=Pi(l,d);p.payload=i,a!=null&&(p.callback=a),i=rr(n,p,d),i!==null&&(ri(i,n,d,l),ko(i,n,d))},enqueueReplaceState:function(n,i,a){n=n._reactInternals;var l=vn(),d=lr(n),p=Pi(l,d);p.tag=1,p.payload=i,a!=null&&(p.callback=a),i=rr(n,p,d),i!==null&&(ri(i,n,d,l),ko(i,n,d))},enqueueForceUpdate:function(n,i){n=n._reactInternals;var a=vn(),l=lr(n),d=Pi(a,l);d.tag=2,i!=null&&(d.callback=i),i=rr(n,d,l),i!==null&&(ri(i,n,l,a),ko(i,n,l))}};function pp(n,i,a,l,d,p,E){return n=n.stateNode,typeof n.shouldComponentUpdate=="function"?n.shouldComponentUpdate(l,p,E):i.prototype&&i.prototype.isPureReactComponent?!wa(a,l)||!wa(d,p):!0}function mp(n,i,a){var l=!1,d=tr,p=i.contextType;return typeof p=="object"&&p!==null?p=Wn(p):(d=An(i)?Rr:cn.current,l=i.contextTypes,p=(l=l!=null)?ps(n,d):tr),i=new i(a,p),n.memoizedState=i.state!==null&&i.state!==void 0?i.state:null,i.updater=Xo,n.stateNode=i,i._reactInternals=n,l&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=d,n.__reactInternalMemoizedMaskedChildContext=p),i}function gp(n,i,a,l){n=i.state,typeof i.componentWillReceiveProps=="function"&&i.componentWillReceiveProps(a,l),typeof i.UNSAFE_componentWillReceiveProps=="function"&&i.UNSAFE_componentWillReceiveProps(a,l),i.state!==n&&Xo.enqueueReplaceState(i,i.state,null)}function vc(n,i,a,l){var d=n.stateNode;d.props=a,d.state=n.memoizedState,d.refs={},rc(n);var p=i.contextType;typeof p=="object"&&p!==null?d.context=Wn(p):(p=An(i)?Rr:cn.current,d.context=ps(n,p)),d.state=n.memoizedState,p=i.getDerivedStateFromProps,typeof p=="function"&&(_c(n,i,p,a),d.state=n.memoizedState),typeof i.getDerivedStateFromProps=="function"||typeof d.getSnapshotBeforeUpdate=="function"||typeof d.UNSAFE_componentWillMount!="function"&&typeof d.componentWillMount!="function"||(i=d.state,typeof d.componentWillMount=="function"&&d.componentWillMount(),typeof d.UNSAFE_componentWillMount=="function"&&d.UNSAFE_componentWillMount(),i!==d.state&&Xo.enqueueReplaceState(d,d.state,null),Bo(n,a,d,l),d.state=n.memoizedState),typeof d.componentDidMount=="function"&&(n.flags|=4194308)}function Ms(n,i){try{var a="",l=i;do a+=de(l),l=l.return;while(l);var d=a}catch(p){d=`
Error generating stack: `+p.message+`
`+p.stack}return{value:n,source:i,stack:d,digest:null}}function xc(n,i,a){return{value:n,source:null,stack:a??null,digest:i??null}}function yc(n,i){try{console.error(i.value)}catch(a){setTimeout(function(){throw a})}}var Av=typeof WeakMap=="function"?WeakMap:Map;function _p(n,i,a){a=Pi(-1,a),a.tag=3,a.payload={element:null};var l=i.value;return a.callback=function(){Jo||(Jo=!0,Uc=l),yc(n,i)},a}function vp(n,i,a){a=Pi(-1,a),a.tag=3;var l=n.type.getDerivedStateFromError;if(typeof l=="function"){var d=i.value;a.payload=function(){return l(d)},a.callback=function(){yc(n,i)}}var p=n.stateNode;return p!==null&&typeof p.componentDidCatch=="function"&&(a.callback=function(){yc(n,i),typeof l!="function"&&(ar===null?ar=new Set([this]):ar.add(this));var E=i.stack;this.componentDidCatch(i.value,{componentStack:E!==null?E:""})}),a}function xp(n,i,a){var l=n.pingCache;if(l===null){l=n.pingCache=new Av;var d=new Set;l.set(i,d)}else d=l.get(i),d===void 0&&(d=new Set,l.set(i,d));d.has(a)||(d.add(a),n=zv.bind(null,n,i,a),i.then(n,n))}function yp(n){do{var i;if((i=n.tag===13)&&(i=n.memoizedState,i=i!==null?i.dehydrated!==null:!0),i)return n;n=n.return}while(n!==null);return null}function Sp(n,i,a,l,d){return(n.mode&1)===0?(n===i?n.flags|=65536:(n.flags|=128,a.flags|=131072,a.flags&=-52805,a.tag===1&&(a.alternate===null?a.tag=17:(i=Pi(-1,1),i.tag=2,rr(a,i,1))),a.lanes|=1),n):(n.flags|=65536,n.lanes=d,n)}var Cv=L.ReactCurrentOwner,Cn=!1;function _n(n,i,a,l){i.child=n===null?Hh(i,null,a,l):vs(i,n.child,a,l)}function Mp(n,i,a,l,d){a=a.render;var p=i.ref;return ys(i,d),l=fc(n,i,a,l,p,d),a=dc(),n!==null&&!Cn?(i.updateQueue=n.updateQueue,i.flags&=-2053,n.lanes&=~d,Li(n,i,d)):(zt&&a&&qu(i),i.flags|=1,_n(n,i,l,d),i.child)}function Ep(n,i,a,l,d){if(n===null){var p=a.type;return typeof p=="function"&&!Vc(p)&&p.defaultProps===void 0&&a.compare===null&&a.defaultProps===void 0?(i.tag=15,i.type=p,wp(n,i,p,l,d)):(n=sl(a.type,null,l,i,i.mode,d),n.ref=i.ref,n.return=i,i.child=n)}if(p=n.child,(n.lanes&d)===0){var E=p.memoizedProps;if(a=a.compare,a=a!==null?a:wa,a(E,l)&&n.ref===i.ref)return Li(n,i,d)}return i.flags|=1,n=cr(p,l),n.ref=i.ref,n.return=i,i.child=n}function wp(n,i,a,l,d){if(n!==null){var p=n.memoizedProps;if(wa(p,l)&&n.ref===i.ref)if(Cn=!1,i.pendingProps=l=p,(n.lanes&d)!==0)(n.flags&131072)!==0&&(Cn=!0);else return i.lanes=n.lanes,Li(n,i,d)}return Sc(n,i,a,l,d)}function Tp(n,i,a){var l=i.pendingProps,d=l.children,p=n!==null?n.memoizedState:null;if(l.mode==="hidden")if((i.mode&1)===0)i.memoizedState={baseLanes:0,cachePool:null,transitions:null},Dt(ws,On),On|=a;else{if((a&1073741824)===0)return n=p!==null?p.baseLanes|a:a,i.lanes=i.childLanes=1073741824,i.memoizedState={baseLanes:n,cachePool:null,transitions:null},i.updateQueue=null,Dt(ws,On),On|=n,null;i.memoizedState={baseLanes:0,cachePool:null,transitions:null},l=p!==null?p.baseLanes:a,Dt(ws,On),On|=l}else p!==null?(l=p.baseLanes|a,i.memoizedState=null):l=a,Dt(ws,On),On|=l;return _n(n,i,d,a),i.child}function Ap(n,i){var a=i.ref;(n===null&&a!==null||n!==null&&n.ref!==a)&&(i.flags|=512,i.flags|=2097152)}function Sc(n,i,a,l,d){var p=An(a)?Rr:cn.current;return p=ps(i,p),ys(i,d),a=fc(n,i,a,l,p,d),l=dc(),n!==null&&!Cn?(i.updateQueue=n.updateQueue,i.flags&=-2053,n.lanes&=~d,Li(n,i,d)):(zt&&l&&qu(i),i.flags|=1,_n(n,i,a,d),i.child)}function Cp(n,i,a,l,d){if(An(a)){var p=!0;Po(i)}else p=!1;if(ys(i,d),i.stateNode===null)qo(n,i),mp(i,a,l),vc(i,a,l,d),l=!0;else if(n===null){var E=i.stateNode,D=i.memoizedProps;E.props=D;var z=E.context,ae=a.contextType;typeof ae=="object"&&ae!==null?ae=Wn(ae):(ae=An(a)?Rr:cn.current,ae=ps(i,ae));var ye=a.getDerivedStateFromProps,we=typeof ye=="function"||typeof E.getSnapshotBeforeUpdate=="function";we||typeof E.UNSAFE_componentWillReceiveProps!="function"&&typeof E.componentWillReceiveProps!="function"||(D!==l||z!==ae)&&gp(i,E,l,ae),ir=!1;var _e=i.memoizedState;E.state=_e,Bo(i,l,E,d),z=i.memoizedState,D!==l||_e!==z||Tn.current||ir?(typeof ye=="function"&&(_c(i,a,ye,l),z=i.memoizedState),(D=ir||pp(i,a,D,l,_e,z,ae))?(we||typeof E.UNSAFE_componentWillMount!="function"&&typeof E.componentWillMount!="function"||(typeof E.componentWillMount=="function"&&E.componentWillMount(),typeof E.UNSAFE_componentWillMount=="function"&&E.UNSAFE_componentWillMount()),typeof E.componentDidMount=="function"&&(i.flags|=4194308)):(typeof E.componentDidMount=="function"&&(i.flags|=4194308),i.memoizedProps=l,i.memoizedState=z),E.props=l,E.state=z,E.context=ae,l=D):(typeof E.componentDidMount=="function"&&(i.flags|=4194308),l=!1)}else{E=i.stateNode,Gh(n,i),D=i.memoizedProps,ae=i.type===i.elementType?D:ti(i.type,D),E.props=ae,we=i.pendingProps,_e=E.context,z=a.contextType,typeof z=="object"&&z!==null?z=Wn(z):(z=An(a)?Rr:cn.current,z=ps(i,z));var He=a.getDerivedStateFromProps;(ye=typeof He=="function"||typeof E.getSnapshotBeforeUpdate=="function")||typeof E.UNSAFE_componentWillReceiveProps!="function"&&typeof E.componentWillReceiveProps!="function"||(D!==we||_e!==z)&&gp(i,E,l,z),ir=!1,_e=i.memoizedState,E.state=_e,Bo(i,l,E,d);var je=i.memoizedState;D!==we||_e!==je||Tn.current||ir?(typeof He=="function"&&(_c(i,a,He,l),je=i.memoizedState),(ae=ir||pp(i,a,ae,l,_e,je,z)||!1)?(ye||typeof E.UNSAFE_componentWillUpdate!="function"&&typeof E.componentWillUpdate!="function"||(typeof E.componentWillUpdate=="function"&&E.componentWillUpdate(l,je,z),typeof E.UNSAFE_componentWillUpdate=="function"&&E.UNSAFE_componentWillUpdate(l,je,z)),typeof E.componentDidUpdate=="function"&&(i.flags|=4),typeof E.getSnapshotBeforeUpdate=="function"&&(i.flags|=1024)):(typeof E.componentDidUpdate!="function"||D===n.memoizedProps&&_e===n.memoizedState||(i.flags|=4),typeof E.getSnapshotBeforeUpdate!="function"||D===n.memoizedProps&&_e===n.memoizedState||(i.flags|=1024),i.memoizedProps=l,i.memoizedState=je),E.props=l,E.state=je,E.context=z,l=ae):(typeof E.componentDidUpdate!="function"||D===n.memoizedProps&&_e===n.memoizedState||(i.flags|=4),typeof E.getSnapshotBeforeUpdate!="function"||D===n.memoizedProps&&_e===n.memoizedState||(i.flags|=1024),l=!1)}return Mc(n,i,a,l,p,d)}function Mc(n,i,a,l,d,p){Ap(n,i);var E=(i.flags&128)!==0;if(!l&&!E)return d&&Nh(i,a,!1),Li(n,i,p);l=i.stateNode,Cv.current=i;var D=E&&typeof a.getDerivedStateFromError!="function"?null:l.render();return i.flags|=1,n!==null&&E?(i.child=vs(i,n.child,null,p),i.child=vs(i,null,D,p)):_n(n,i,D,p),i.memoizedState=l.state,d&&Nh(i,a,!0),i.child}function Rp(n){var i=n.stateNode;i.pendingContext?Ph(n,i.pendingContext,i.pendingContext!==i.context):i.context&&Ph(n,i.context,!1),sc(n,i.containerInfo)}function bp(n,i,a,l,d){return _s(),Qu(d),i.flags|=256,_n(n,i,a,l),i.child}var Ec={dehydrated:null,treeContext:null,retryLane:0};function wc(n){return{baseLanes:n,cachePool:null,transitions:null}}function Pp(n,i,a){var l=i.pendingProps,d=Ht.current,p=!1,E=(i.flags&128)!==0,D;if((D=E)||(D=n!==null&&n.memoizedState===null?!1:(d&2)!==0),D?(p=!0,i.flags&=-129):(n===null||n.memoizedState!==null)&&(d|=1),Dt(Ht,d&1),n===null)return Zu(i),n=i.memoizedState,n!==null&&(n=n.dehydrated,n!==null)?((i.mode&1)===0?i.lanes=1:n.data==="$!"?i.lanes=8:i.lanes=1073741824,null):(E=l.children,n=l.fallback,p?(l=i.mode,p=i.child,E={mode:"hidden",children:E},(l&1)===0&&p!==null?(p.childLanes=0,p.pendingProps=E):p=al(E,l,0,null),n=kr(n,l,a,null),p.return=i,n.return=i,p.sibling=n,i.child=p,i.child.memoizedState=wc(a),i.memoizedState=Ec,n):Tc(i,E));if(d=n.memoizedState,d!==null&&(D=d.dehydrated,D!==null))return Rv(n,i,E,l,D,d,a);if(p){p=l.fallback,E=i.mode,d=n.child,D=d.sibling;var z={mode:"hidden",children:l.children};return(E&1)===0&&i.child!==d?(l=i.child,l.childLanes=0,l.pendingProps=z,i.deletions=null):(l=cr(d,z),l.subtreeFlags=d.subtreeFlags&14680064),D!==null?p=cr(D,p):(p=kr(p,E,a,null),p.flags|=2),p.return=i,l.return=i,l.sibling=p,i.child=l,l=p,p=i.child,E=n.child.memoizedState,E=E===null?wc(a):{baseLanes:E.baseLanes|a,cachePool:null,transitions:E.transitions},p.memoizedState=E,p.childLanes=n.childLanes&~a,i.memoizedState=Ec,l}return p=n.child,n=p.sibling,l=cr(p,{mode:"visible",children:l.children}),(i.mode&1)===0&&(l.lanes=a),l.return=i,l.sibling=null,n!==null&&(a=i.deletions,a===null?(i.deletions=[n],i.flags|=16):a.push(n)),i.child=l,i.memoizedState=null,l}function Tc(n,i){return i=al({mode:"visible",children:i},n.mode,0,null),i.return=n,n.child=i}function Yo(n,i,a,l){return l!==null&&Qu(l),vs(i,n.child,null,a),n=Tc(i,i.pendingProps.children),n.flags|=2,i.memoizedState=null,n}function Rv(n,i,a,l,d,p,E){if(a)return i.flags&256?(i.flags&=-257,l=xc(Error(t(422))),Yo(n,i,E,l)):i.memoizedState!==null?(i.child=n.child,i.flags|=128,null):(p=l.fallback,d=i.mode,l=al({mode:"visible",children:l.children},d,0,null),p=kr(p,d,E,null),p.flags|=2,l.return=i,p.return=i,l.sibling=p,i.child=l,(i.mode&1)!==0&&vs(i,n.child,null,E),i.child.memoizedState=wc(E),i.memoizedState=Ec,p);if((i.mode&1)===0)return Yo(n,i,E,null);if(d.data==="$!"){if(l=d.nextSibling&&d.nextSibling.dataset,l)var D=l.dgst;return l=D,p=Error(t(419)),l=xc(p,l,void 0),Yo(n,i,E,l)}if(D=(E&n.childLanes)!==0,Cn||D){if(l=sn,l!==null){switch(E&-E){case 4:d=2;break;case 16:d=8;break;case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:d=32;break;case 536870912:d=268435456;break;default:d=0}d=(d&(l.suspendedLanes|E))!==0?0:d,d!==0&&d!==p.retryLane&&(p.retryLane=d,bi(n,d),ri(l,n,d,-1))}return Hc(),l=xc(Error(t(421))),Yo(n,i,E,l)}return d.data==="$?"?(i.flags|=128,i.child=n.child,i=Hv.bind(null,n),d._reactRetry=i,null):(n=p.treeContext,Fn=Ji(d.nextSibling),Un=i,zt=!0,ei=null,n!==null&&(Vn[Gn++]=Ci,Vn[Gn++]=Ri,Vn[Gn++]=br,Ci=n.id,Ri=n.overflow,br=i),i=Tc(i,l.children),i.flags|=4096,i)}function Lp(n,i,a){n.lanes|=i;var l=n.alternate;l!==null&&(l.lanes|=i),nc(n.return,i,a)}function Ac(n,i,a,l,d){var p=n.memoizedState;p===null?n.memoizedState={isBackwards:i,rendering:null,renderingStartTime:0,last:l,tail:a,tailMode:d}:(p.isBackwards=i,p.rendering=null,p.renderingStartTime=0,p.last=l,p.tail=a,p.tailMode=d)}function Np(n,i,a){var l=i.pendingProps,d=l.revealOrder,p=l.tail;if(_n(n,i,l.children,a),l=Ht.current,(l&2)!==0)l=l&1|2,i.flags|=128;else{if(n!==null&&(n.flags&128)!==0)e:for(n=i.child;n!==null;){if(n.tag===13)n.memoizedState!==null&&Lp(n,a,i);else if(n.tag===19)Lp(n,a,i);else if(n.child!==null){n.child.return=n,n=n.child;continue}if(n===i)break e;for(;n.sibling===null;){if(n.return===null||n.return===i)break e;n=n.return}n.sibling.return=n.return,n=n.sibling}l&=1}if(Dt(Ht,l),(i.mode&1)===0)i.memoizedState=null;else switch(d){case"forwards":for(a=i.child,d=null;a!==null;)n=a.alternate,n!==null&&zo(n)===null&&(d=a),a=a.sibling;a=d,a===null?(d=i.child,i.child=null):(d=a.sibling,a.sibling=null),Ac(i,!1,d,a,p);break;case"backwards":for(a=null,d=i.child,i.child=null;d!==null;){if(n=d.alternate,n!==null&&zo(n)===null){i.child=d;break}n=d.sibling,d.sibling=a,a=d,d=n}Ac(i,!0,a,null,p);break;case"together":Ac(i,!1,null,null,void 0);break;default:i.memoizedState=null}return i.child}function qo(n,i){(i.mode&1)===0&&n!==null&&(n.alternate=null,i.alternate=null,i.flags|=2)}function Li(n,i,a){if(n!==null&&(i.dependencies=n.dependencies),Ir|=i.lanes,(a&i.childLanes)===0)return null;if(n!==null&&i.child!==n.child)throw Error(t(153));if(i.child!==null){for(n=i.child,a=cr(n,n.pendingProps),i.child=a,a.return=i;n.sibling!==null;)n=n.sibling,a=a.sibling=cr(n,n.pendingProps),a.return=i;a.sibling=null}return i.child}function bv(n,i,a){switch(i.tag){case 3:Rp(i),_s();break;case 5:Xh(i);break;case 1:An(i.type)&&Po(i);break;case 4:sc(i,i.stateNode.containerInfo);break;case 10:var l=i.type._context,d=i.memoizedProps.value;Dt(Fo,l._currentValue),l._currentValue=d;break;case 13:if(l=i.memoizedState,l!==null)return l.dehydrated!==null?(Dt(Ht,Ht.current&1),i.flags|=128,null):(a&i.child.childLanes)!==0?Pp(n,i,a):(Dt(Ht,Ht.current&1),n=Li(n,i,a),n!==null?n.sibling:null);Dt(Ht,Ht.current&1);break;case 19:if(l=(a&i.childLanes)!==0,(n.flags&128)!==0){if(l)return Np(n,i,a);i.flags|=128}if(d=i.memoizedState,d!==null&&(d.rendering=null,d.tail=null,d.lastEffect=null),Dt(Ht,Ht.current),l)break;return null;case 22:case 23:return i.lanes=0,Tp(n,i,a)}return Li(n,i,a)}var Dp,Cc,Ip,Up;Dp=function(n,i){for(var a=i.child;a!==null;){if(a.tag===5||a.tag===6)n.appendChild(a.stateNode);else if(a.tag!==4&&a.child!==null){a.child.return=a,a=a.child;continue}if(a===i)break;for(;a.sibling===null;){if(a.return===null||a.return===i)return;a=a.return}a.sibling.return=a.return,a=a.sibling}},Cc=function(){},Ip=function(n,i,a,l){var d=n.memoizedProps;if(d!==l){n=i.stateNode,Nr(mi.current);var p=null;switch(a){case"input":d=gt(n,d),l=gt(n,l),p=[];break;case"select":d=fe({},d,{value:void 0}),l=fe({},l,{value:void 0}),p=[];break;case"textarea":d=oe(n,d),l=oe(n,l),p=[];break;default:typeof d.onClick!="function"&&typeof l.onClick=="function"&&(n.onclick=Co)}ft(a,l);var E;a=null;for(ae in d)if(!l.hasOwnProperty(ae)&&d.hasOwnProperty(ae)&&d[ae]!=null)if(ae==="style"){var D=d[ae];for(E in D)D.hasOwnProperty(E)&&(a||(a={}),a[E]="")}else ae!=="dangerouslySetInnerHTML"&&ae!=="children"&&ae!=="suppressContentEditableWarning"&&ae!=="suppressHydrationWarning"&&ae!=="autoFocus"&&(o.hasOwnProperty(ae)?p||(p=[]):(p=p||[]).push(ae,null));for(ae in l){var z=l[ae];if(D=d!=null?d[ae]:void 0,l.hasOwnProperty(ae)&&z!==D&&(z!=null||D!=null))if(ae==="style")if(D){for(E in D)!D.hasOwnProperty(E)||z&&z.hasOwnProperty(E)||(a||(a={}),a[E]="");for(E in z)z.hasOwnProperty(E)&&D[E]!==z[E]&&(a||(a={}),a[E]=z[E])}else a||(p||(p=[]),p.push(ae,a)),a=z;else ae==="dangerouslySetInnerHTML"?(z=z?z.__html:void 0,D=D?D.__html:void 0,z!=null&&D!==z&&(p=p||[]).push(ae,z)):ae==="children"?typeof z!="string"&&typeof z!="number"||(p=p||[]).push(ae,""+z):ae!=="suppressContentEditableWarning"&&ae!=="suppressHydrationWarning"&&(o.hasOwnProperty(ae)?(z!=null&&ae==="onScroll"&&Ut("scroll",n),p||D===z||(p=[])):(p=p||[]).push(ae,z))}a&&(p=p||[]).push("style",a);var ae=p;(i.updateQueue=ae)&&(i.flags|=4)}},Up=function(n,i,a,l){a!==l&&(i.flags|=4)};function Ba(n,i){if(!zt)switch(n.tailMode){case"hidden":i=n.tail;for(var a=null;i!==null;)i.alternate!==null&&(a=i),i=i.sibling;a===null?n.tail=null:a.sibling=null;break;case"collapsed":a=n.tail;for(var l=null;a!==null;)a.alternate!==null&&(l=a),a=a.sibling;l===null?i||n.tail===null?n.tail=null:n.tail.sibling=null:l.sibling=null}}function dn(n){var i=n.alternate!==null&&n.alternate.child===n.child,a=0,l=0;if(i)for(var d=n.child;d!==null;)a|=d.lanes|d.childLanes,l|=d.subtreeFlags&14680064,l|=d.flags&14680064,d.return=n,d=d.sibling;else for(d=n.child;d!==null;)a|=d.lanes|d.childLanes,l|=d.subtreeFlags,l|=d.flags,d.return=n,d=d.sibling;return n.subtreeFlags|=l,n.childLanes=a,i}function Pv(n,i,a){var l=i.pendingProps;switch($u(i),i.tag){case 2:case 16:case 15:case 0:case 11:case 7:case 8:case 12:case 9:case 14:return dn(i),null;case 1:return An(i.type)&&bo(),dn(i),null;case 3:return l=i.stateNode,Ss(),Ft(Tn),Ft(cn),lc(),l.pendingContext&&(l.context=l.pendingContext,l.pendingContext=null),(n===null||n.child===null)&&(Io(i)?i.flags|=4:n===null||n.memoizedState.isDehydrated&&(i.flags&256)===0||(i.flags|=1024,ei!==null&&(kc(ei),ei=null))),Cc(n,i),dn(i),null;case 5:ac(i);var d=Nr(Ia.current);if(a=i.type,n!==null&&i.stateNode!=null)Ip(n,i,a,l,d),n.ref!==i.ref&&(i.flags|=512,i.flags|=2097152);else{if(!l){if(i.stateNode===null)throw Error(t(166));return dn(i),null}if(n=Nr(mi.current),Io(i)){l=i.stateNode,a=i.type;var p=i.memoizedProps;switch(l[pi]=i,l[ba]=p,n=(i.mode&1)!==0,a){case"dialog":Ut("cancel",l),Ut("close",l);break;case"iframe":case"object":case"embed":Ut("load",l);break;case"video":case"audio":for(d=0;d<Aa.length;d++)Ut(Aa[d],l);break;case"source":Ut("error",l);break;case"img":case"image":case"link":Ut("error",l),Ut("load",l);break;case"details":Ut("toggle",l);break;case"input":ne(l,p),Ut("invalid",l);break;case"select":l._wrapperState={wasMultiple:!!p.multiple},Ut("invalid",l);break;case"textarea":ve(l,p),Ut("invalid",l)}ft(a,p),d=null;for(var E in p)if(p.hasOwnProperty(E)){var D=p[E];E==="children"?typeof D=="string"?l.textContent!==D&&(p.suppressHydrationWarning!==!0&&Ao(l.textContent,D,n),d=["children",D]):typeof D=="number"&&l.textContent!==""+D&&(p.suppressHydrationWarning!==!0&&Ao(l.textContent,D,n),d=["children",""+D]):o.hasOwnProperty(E)&&D!=null&&E==="onScroll"&&Ut("scroll",l)}switch(a){case"input":Et(l),Ae(l,p,!0);break;case"textarea":Et(l),ge(l);break;case"select":case"option":break;default:typeof p.onClick=="function"&&(l.onclick=Co)}l=d,i.updateQueue=l,l!==null&&(i.flags|=4)}else{E=d.nodeType===9?d:d.ownerDocument,n==="http://www.w3.org/1999/xhtml"&&(n=Ze(a)),n==="http://www.w3.org/1999/xhtml"?a==="script"?(n=E.createElement("div"),n.innerHTML="<script><\/script>",n=n.removeChild(n.firstChild)):typeof l.is=="string"?n=E.createElement(a,{is:l.is}):(n=E.createElement(a),a==="select"&&(E=n,l.multiple?E.multiple=!0:l.size&&(E.size=l.size))):n=E.createElementNS(n,a),n[pi]=i,n[ba]=l,Dp(n,i,!1,!1),i.stateNode=n;e:{switch(E=Ct(a,l),a){case"dialog":Ut("cancel",n),Ut("close",n),d=l;break;case"iframe":case"object":case"embed":Ut("load",n),d=l;break;case"video":case"audio":for(d=0;d<Aa.length;d++)Ut(Aa[d],n);d=l;break;case"source":Ut("error",n),d=l;break;case"img":case"image":case"link":Ut("error",n),Ut("load",n),d=l;break;case"details":Ut("toggle",n),d=l;break;case"input":ne(n,l),d=gt(n,l),Ut("invalid",n);break;case"option":d=l;break;case"select":n._wrapperState={wasMultiple:!!l.multiple},d=fe({},l,{value:void 0}),Ut("invalid",n);break;case"textarea":ve(n,l),d=oe(n,l),Ut("invalid",n);break;default:d=l}ft(a,d),D=d;for(p in D)if(D.hasOwnProperty(p)){var z=D[p];p==="style"?Ve(n,z):p==="dangerouslySetInnerHTML"?(z=z?z.__html:void 0,z!=null&&ct(n,z)):p==="children"?typeof z=="string"?(a!=="textarea"||z!=="")&&Te(n,z):typeof z=="number"&&Te(n,""+z):p!=="suppressContentEditableWarning"&&p!=="suppressHydrationWarning"&&p!=="autoFocus"&&(o.hasOwnProperty(p)?z!=null&&p==="onScroll"&&Ut("scroll",n):z!=null&&R(n,p,z,E))}switch(a){case"input":Et(n),Ae(n,l,!1);break;case"textarea":Et(n),ge(n);break;case"option":l.value!=null&&n.setAttribute("value",""+Ie(l.value));break;case"select":n.multiple=!!l.multiple,p=l.value,p!=null?A(n,!!l.multiple,p,!1):l.defaultValue!=null&&A(n,!!l.multiple,l.defaultValue,!0);break;default:typeof d.onClick=="function"&&(n.onclick=Co)}switch(a){case"button":case"input":case"select":case"textarea":l=!!l.autoFocus;break e;case"img":l=!0;break e;default:l=!1}}l&&(i.flags|=4)}i.ref!==null&&(i.flags|=512,i.flags|=2097152)}return dn(i),null;case 6:if(n&&i.stateNode!=null)Up(n,i,n.memoizedProps,l);else{if(typeof l!="string"&&i.stateNode===null)throw Error(t(166));if(a=Nr(Ia.current),Nr(mi.current),Io(i)){if(l=i.stateNode,a=i.memoizedProps,l[pi]=i,(p=l.nodeValue!==a)&&(n=Un,n!==null))switch(n.tag){case 3:Ao(l.nodeValue,a,(n.mode&1)!==0);break;case 5:n.memoizedProps.suppressHydrationWarning!==!0&&Ao(l.nodeValue,a,(n.mode&1)!==0)}p&&(i.flags|=4)}else l=(a.nodeType===9?a:a.ownerDocument).createTextNode(l),l[pi]=i,i.stateNode=l}return dn(i),null;case 13:if(Ft(Ht),l=i.memoizedState,n===null||n.memoizedState!==null&&n.memoizedState.dehydrated!==null){if(zt&&Fn!==null&&(i.mode&1)!==0&&(i.flags&128)===0)kh(),_s(),i.flags|=98560,p=!1;else if(p=Io(i),l!==null&&l.dehydrated!==null){if(n===null){if(!p)throw Error(t(318));if(p=i.memoizedState,p=p!==null?p.dehydrated:null,!p)throw Error(t(317));p[pi]=i}else _s(),(i.flags&128)===0&&(i.memoizedState=null),i.flags|=4;dn(i),p=!1}else ei!==null&&(kc(ei),ei=null),p=!0;if(!p)return i.flags&65536?i:null}return(i.flags&128)!==0?(i.lanes=a,i):(l=l!==null,l!==(n!==null&&n.memoizedState!==null)&&l&&(i.child.flags|=8192,(i.mode&1)!==0&&(n===null||(Ht.current&1)!==0?Kt===0&&(Kt=3):Hc())),i.updateQueue!==null&&(i.flags|=4),dn(i),null);case 4:return Ss(),Cc(n,i),n===null&&Ca(i.stateNode.containerInfo),dn(i),null;case 10:return tc(i.type._context),dn(i),null;case 17:return An(i.type)&&bo(),dn(i),null;case 19:if(Ft(Ht),p=i.memoizedState,p===null)return dn(i),null;if(l=(i.flags&128)!==0,E=p.rendering,E===null)if(l)Ba(p,!1);else{if(Kt!==0||n!==null&&(n.flags&128)!==0)for(n=i.child;n!==null;){if(E=zo(n),E!==null){for(i.flags|=128,Ba(p,!1),l=E.updateQueue,l!==null&&(i.updateQueue=l,i.flags|=4),i.subtreeFlags=0,l=a,a=i.child;a!==null;)p=a,n=l,p.flags&=14680066,E=p.alternate,E===null?(p.childLanes=0,p.lanes=n,p.child=null,p.subtreeFlags=0,p.memoizedProps=null,p.memoizedState=null,p.updateQueue=null,p.dependencies=null,p.stateNode=null):(p.childLanes=E.childLanes,p.lanes=E.lanes,p.child=E.child,p.subtreeFlags=0,p.deletions=null,p.memoizedProps=E.memoizedProps,p.memoizedState=E.memoizedState,p.updateQueue=E.updateQueue,p.type=E.type,n=E.dependencies,p.dependencies=n===null?null:{lanes:n.lanes,firstContext:n.firstContext}),a=a.sibling;return Dt(Ht,Ht.current&1|2),i.child}n=n.sibling}p.tail!==null&&Le()>Ts&&(i.flags|=128,l=!0,Ba(p,!1),i.lanes=4194304)}else{if(!l)if(n=zo(E),n!==null){if(i.flags|=128,l=!0,a=n.updateQueue,a!==null&&(i.updateQueue=a,i.flags|=4),Ba(p,!0),p.tail===null&&p.tailMode==="hidden"&&!E.alternate&&!zt)return dn(i),null}else 2*Le()-p.renderingStartTime>Ts&&a!==1073741824&&(i.flags|=128,l=!0,Ba(p,!1),i.lanes=4194304);p.isBackwards?(E.sibling=i.child,i.child=E):(a=p.last,a!==null?a.sibling=E:i.child=E,p.last=E)}return p.tail!==null?(i=p.tail,p.rendering=i,p.tail=i.sibling,p.renderingStartTime=Le(),i.sibling=null,a=Ht.current,Dt(Ht,l?a&1|2:a&1),i):(dn(i),null);case 22:case 23:return zc(),l=i.memoizedState!==null,n!==null&&n.memoizedState!==null!==l&&(i.flags|=8192),l&&(i.mode&1)!==0?(On&1073741824)!==0&&(dn(i),i.subtreeFlags&6&&(i.flags|=8192)):dn(i),null;case 24:return null;case 25:return null}throw Error(t(156,i.tag))}function Lv(n,i){switch($u(i),i.tag){case 1:return An(i.type)&&bo(),n=i.flags,n&65536?(i.flags=n&-65537|128,i):null;case 3:return Ss(),Ft(Tn),Ft(cn),lc(),n=i.flags,(n&65536)!==0&&(n&128)===0?(i.flags=n&-65537|128,i):null;case 5:return ac(i),null;case 13:if(Ft(Ht),n=i.memoizedState,n!==null&&n.dehydrated!==null){if(i.alternate===null)throw Error(t(340));_s()}return n=i.flags,n&65536?(i.flags=n&-65537|128,i):null;case 19:return Ft(Ht),null;case 4:return Ss(),null;case 10:return tc(i.type._context),null;case 22:case 23:return zc(),null;case 24:return null;default:return null}}var $o=!1,hn=!1,Nv=typeof WeakSet=="function"?WeakSet:Set,We=null;function Es(n,i){var a=n.ref;if(a!==null)if(typeof a=="function")try{a(null)}catch(l){Wt(n,i,l)}else a.current=null}function Rc(n,i,a){try{a()}catch(l){Wt(n,i,l)}}var Fp=!1;function Dv(n,i){if(zu=mo,n=ph(),Nu(n)){if("selectionStart"in n)var a={start:n.selectionStart,end:n.selectionEnd};else e:{a=(a=n.ownerDocument)&&a.defaultView||window;var l=a.getSelection&&a.getSelection();if(l&&l.rangeCount!==0){a=l.anchorNode;var d=l.anchorOffset,p=l.focusNode;l=l.focusOffset;try{a.nodeType,p.nodeType}catch{a=null;break e}var E=0,D=-1,z=-1,ae=0,ye=0,we=n,_e=null;t:for(;;){for(var He;we!==a||d!==0&&we.nodeType!==3||(D=E+d),we!==p||l!==0&&we.nodeType!==3||(z=E+l),we.nodeType===3&&(E+=we.nodeValue.length),(He=we.firstChild)!==null;)_e=we,we=He;for(;;){if(we===n)break t;if(_e===a&&++ae===d&&(D=E),_e===p&&++ye===l&&(z=E),(He=we.nextSibling)!==null)break;we=_e,_e=we.parentNode}we=He}a=D===-1||z===-1?null:{start:D,end:z}}else a=null}a=a||{start:0,end:0}}else a=null;for(Hu={focusedElem:n,selectionRange:a},mo=!1,We=i;We!==null;)if(i=We,n=i.child,(i.subtreeFlags&1028)!==0&&n!==null)n.return=i,We=n;else for(;We!==null;){i=We;try{var je=i.alternate;if((i.flags&1024)!==0)switch(i.tag){case 0:case 11:case 15:break;case 1:if(je!==null){var Ye=je.memoizedProps,jt=je.memoizedState,$=i.stateNode,W=$.getSnapshotBeforeUpdate(i.elementType===i.type?Ye:ti(i.type,Ye),jt);$.__reactInternalSnapshotBeforeUpdate=W}break;case 3:var ee=i.stateNode.containerInfo;ee.nodeType===1?ee.textContent="":ee.nodeType===9&&ee.documentElement&&ee.removeChild(ee.documentElement);break;case 5:case 6:case 4:case 17:break;default:throw Error(t(163))}}catch(be){Wt(i,i.return,be)}if(n=i.sibling,n!==null){n.return=i.return,We=n;break}We=i.return}return je=Fp,Fp=!1,je}function za(n,i,a){var l=i.updateQueue;if(l=l!==null?l.lastEffect:null,l!==null){var d=l=l.next;do{if((d.tag&n)===n){var p=d.destroy;d.destroy=void 0,p!==void 0&&Rc(i,a,p)}d=d.next}while(d!==l)}}function Ko(n,i){if(i=i.updateQueue,i=i!==null?i.lastEffect:null,i!==null){var a=i=i.next;do{if((a.tag&n)===n){var l=a.create;a.destroy=l()}a=a.next}while(a!==i)}}function bc(n){var i=n.ref;if(i!==null){var a=n.stateNode;switch(n.tag){case 5:n=a;break;default:n=a}typeof i=="function"?i(n):i.current=n}}function Op(n){var i=n.alternate;i!==null&&(n.alternate=null,Op(i)),n.child=null,n.deletions=null,n.sibling=null,n.tag===5&&(i=n.stateNode,i!==null&&(delete i[pi],delete i[ba],delete i[ju],delete i[mv],delete i[gv])),n.stateNode=null,n.return=null,n.dependencies=null,n.memoizedProps=null,n.memoizedState=null,n.pendingProps=null,n.stateNode=null,n.updateQueue=null}function kp(n){return n.tag===5||n.tag===3||n.tag===4}function Bp(n){e:for(;;){for(;n.sibling===null;){if(n.return===null||kp(n.return))return null;n=n.return}for(n.sibling.return=n.return,n=n.sibling;n.tag!==5&&n.tag!==6&&n.tag!==18;){if(n.flags&2||n.child===null||n.tag===4)continue e;n.child.return=n,n=n.child}if(!(n.flags&2))return n.stateNode}}function Pc(n,i,a){var l=n.tag;if(l===5||l===6)n=n.stateNode,i?a.nodeType===8?a.parentNode.insertBefore(n,i):a.insertBefore(n,i):(a.nodeType===8?(i=a.parentNode,i.insertBefore(n,a)):(i=a,i.appendChild(n)),a=a._reactRootContainer,a!=null||i.onclick!==null||(i.onclick=Co));else if(l!==4&&(n=n.child,n!==null))for(Pc(n,i,a),n=n.sibling;n!==null;)Pc(n,i,a),n=n.sibling}function Lc(n,i,a){var l=n.tag;if(l===5||l===6)n=n.stateNode,i?a.insertBefore(n,i):a.appendChild(n);else if(l!==4&&(n=n.child,n!==null))for(Lc(n,i,a),n=n.sibling;n!==null;)Lc(n,i,a),n=n.sibling}var ln=null,ni=!1;function sr(n,i,a){for(a=a.child;a!==null;)zp(n,i,a),a=a.sibling}function zp(n,i,a){if(dt&&typeof dt.onCommitFiberUnmount=="function")try{dt.onCommitFiberUnmount(tn,a)}catch{}switch(a.tag){case 5:hn||Es(a,i);case 6:var l=ln,d=ni;ln=null,sr(n,i,a),ln=l,ni=d,ln!==null&&(ni?(n=ln,a=a.stateNode,n.nodeType===8?n.parentNode.removeChild(a):n.removeChild(a)):ln.removeChild(a.stateNode));break;case 18:ln!==null&&(ni?(n=ln,a=a.stateNode,n.nodeType===8?Wu(n.parentNode,a):n.nodeType===1&&Wu(n,a),va(n)):Wu(ln,a.stateNode));break;case 4:l=ln,d=ni,ln=a.stateNode.containerInfo,ni=!0,sr(n,i,a),ln=l,ni=d;break;case 0:case 11:case 14:case 15:if(!hn&&(l=a.updateQueue,l!==null&&(l=l.lastEffect,l!==null))){d=l=l.next;do{var p=d,E=p.destroy;p=p.tag,E!==void 0&&((p&2)!==0||(p&4)!==0)&&Rc(a,i,E),d=d.next}while(d!==l)}sr(n,i,a);break;case 1:if(!hn&&(Es(a,i),l=a.stateNode,typeof l.componentWillUnmount=="function"))try{l.props=a.memoizedProps,l.state=a.memoizedState,l.componentWillUnmount()}catch(D){Wt(a,i,D)}sr(n,i,a);break;case 21:sr(n,i,a);break;case 22:a.mode&1?(hn=(l=hn)||a.memoizedState!==null,sr(n,i,a),hn=l):sr(n,i,a);break;default:sr(n,i,a)}}function Hp(n){var i=n.updateQueue;if(i!==null){n.updateQueue=null;var a=n.stateNode;a===null&&(a=n.stateNode=new Nv),i.forEach(function(l){var d=Vv.bind(null,n,l);a.has(l)||(a.add(l),l.then(d,d))})}}function ii(n,i){var a=i.deletions;if(a!==null)for(var l=0;l<a.length;l++){var d=a[l];try{var p=n,E=i,D=E;e:for(;D!==null;){switch(D.tag){case 5:ln=D.stateNode,ni=!1;break e;case 3:ln=D.stateNode.containerInfo,ni=!0;break e;case 4:ln=D.stateNode.containerInfo,ni=!0;break e}D=D.return}if(ln===null)throw Error(t(160));zp(p,E,d),ln=null,ni=!1;var z=d.alternate;z!==null&&(z.return=null),d.return=null}catch(ae){Wt(d,i,ae)}}if(i.subtreeFlags&12854)for(i=i.child;i!==null;)Vp(i,n),i=i.sibling}function Vp(n,i){var a=n.alternate,l=n.flags;switch(n.tag){case 0:case 11:case 14:case 15:if(ii(i,n),_i(n),l&4){try{za(3,n,n.return),Ko(3,n)}catch(Ye){Wt(n,n.return,Ye)}try{za(5,n,n.return)}catch(Ye){Wt(n,n.return,Ye)}}break;case 1:ii(i,n),_i(n),l&512&&a!==null&&Es(a,a.return);break;case 5:if(ii(i,n),_i(n),l&512&&a!==null&&Es(a,a.return),n.flags&32){var d=n.stateNode;try{Te(d,"")}catch(Ye){Wt(n,n.return,Ye)}}if(l&4&&(d=n.stateNode,d!=null)){var p=n.memoizedProps,E=a!==null?a.memoizedProps:p,D=n.type,z=n.updateQueue;if(n.updateQueue=null,z!==null)try{D==="input"&&p.type==="radio"&&p.name!=null&&ie(d,p),Ct(D,E);var ae=Ct(D,p);for(E=0;E<z.length;E+=2){var ye=z[E],we=z[E+1];ye==="style"?Ve(d,we):ye==="dangerouslySetInnerHTML"?ct(d,we):ye==="children"?Te(d,we):R(d,ye,we,ae)}switch(D){case"input":De(d,p);break;case"textarea":Ee(d,p);break;case"select":var _e=d._wrapperState.wasMultiple;d._wrapperState.wasMultiple=!!p.multiple;var He=p.value;He!=null?A(d,!!p.multiple,He,!1):_e!==!!p.multiple&&(p.defaultValue!=null?A(d,!!p.multiple,p.defaultValue,!0):A(d,!!p.multiple,p.multiple?[]:"",!1))}d[ba]=p}catch(Ye){Wt(n,n.return,Ye)}}break;case 6:if(ii(i,n),_i(n),l&4){if(n.stateNode===null)throw Error(t(162));d=n.stateNode,p=n.memoizedProps;try{d.nodeValue=p}catch(Ye){Wt(n,n.return,Ye)}}break;case 3:if(ii(i,n),_i(n),l&4&&a!==null&&a.memoizedState.isDehydrated)try{va(i.containerInfo)}catch(Ye){Wt(n,n.return,Ye)}break;case 4:ii(i,n),_i(n);break;case 13:ii(i,n),_i(n),d=n.child,d.flags&8192&&(p=d.memoizedState!==null,d.stateNode.isHidden=p,!p||d.alternate!==null&&d.alternate.memoizedState!==null||(Ic=Le())),l&4&&Hp(n);break;case 22:if(ye=a!==null&&a.memoizedState!==null,n.mode&1?(hn=(ae=hn)||ye,ii(i,n),hn=ae):ii(i,n),_i(n),l&8192){if(ae=n.memoizedState!==null,(n.stateNode.isHidden=ae)&&!ye&&(n.mode&1)!==0)for(We=n,ye=n.child;ye!==null;){for(we=We=ye;We!==null;){switch(_e=We,He=_e.child,_e.tag){case 0:case 11:case 14:case 15:za(4,_e,_e.return);break;case 1:Es(_e,_e.return);var je=_e.stateNode;if(typeof je.componentWillUnmount=="function"){l=_e,a=_e.return;try{i=l,je.props=i.memoizedProps,je.state=i.memoizedState,je.componentWillUnmount()}catch(Ye){Wt(l,a,Ye)}}break;case 5:Es(_e,_e.return);break;case 22:if(_e.memoizedState!==null){jp(we);continue}}He!==null?(He.return=_e,We=He):jp(we)}ye=ye.sibling}e:for(ye=null,we=n;;){if(we.tag===5){if(ye===null){ye=we;try{d=we.stateNode,ae?(p=d.style,typeof p.setProperty=="function"?p.setProperty("display","none","important"):p.display="none"):(D=we.stateNode,z=we.memoizedProps.style,E=z!=null&&z.hasOwnProperty("display")?z.display:null,D.style.display=rt("display",E))}catch(Ye){Wt(n,n.return,Ye)}}}else if(we.tag===6){if(ye===null)try{we.stateNode.nodeValue=ae?"":we.memoizedProps}catch(Ye){Wt(n,n.return,Ye)}}else if((we.tag!==22&&we.tag!==23||we.memoizedState===null||we===n)&&we.child!==null){we.child.return=we,we=we.child;continue}if(we===n)break e;for(;we.sibling===null;){if(we.return===null||we.return===n)break e;ye===we&&(ye=null),we=we.return}ye===we&&(ye=null),we.sibling.return=we.return,we=we.sibling}}break;case 19:ii(i,n),_i(n),l&4&&Hp(n);break;case 21:break;default:ii(i,n),_i(n)}}function _i(n){var i=n.flags;if(i&2){try{e:{for(var a=n.return;a!==null;){if(kp(a)){var l=a;break e}a=a.return}throw Error(t(160))}switch(l.tag){case 5:var d=l.stateNode;l.flags&32&&(Te(d,""),l.flags&=-33);var p=Bp(n);Lc(n,p,d);break;case 3:case 4:var E=l.stateNode.containerInfo,D=Bp(n);Pc(n,D,E);break;default:throw Error(t(161))}}catch(z){Wt(n,n.return,z)}n.flags&=-3}i&4096&&(n.flags&=-4097)}function Iv(n,i,a){We=n,Gp(n)}function Gp(n,i,a){for(var l=(n.mode&1)!==0;We!==null;){var d=We,p=d.child;if(d.tag===22&&l){var E=d.memoizedState!==null||$o;if(!E){var D=d.alternate,z=D!==null&&D.memoizedState!==null||hn;D=$o;var ae=hn;if($o=E,(hn=z)&&!ae)for(We=d;We!==null;)E=We,z=E.child,E.tag===22&&E.memoizedState!==null?Xp(d):z!==null?(z.return=E,We=z):Xp(d);for(;p!==null;)We=p,Gp(p),p=p.sibling;We=d,$o=D,hn=ae}Wp(n)}else(d.subtreeFlags&8772)!==0&&p!==null?(p.return=d,We=p):Wp(n)}}function Wp(n){for(;We!==null;){var i=We;if((i.flags&8772)!==0){var a=i.alternate;try{if((i.flags&8772)!==0)switch(i.tag){case 0:case 11:case 15:hn||Ko(5,i);break;case 1:var l=i.stateNode;if(i.flags&4&&!hn)if(a===null)l.componentDidMount();else{var d=i.elementType===i.type?a.memoizedProps:ti(i.type,a.memoizedProps);l.componentDidUpdate(d,a.memoizedState,l.__reactInternalSnapshotBeforeUpdate)}var p=i.updateQueue;p!==null&&jh(i,p,l);break;case 3:var E=i.updateQueue;if(E!==null){if(a=null,i.child!==null)switch(i.child.tag){case 5:a=i.child.stateNode;break;case 1:a=i.child.stateNode}jh(i,E,a)}break;case 5:var D=i.stateNode;if(a===null&&i.flags&4){a=D;var z=i.memoizedProps;switch(i.type){case"button":case"input":case"select":case"textarea":z.autoFocus&&a.focus();break;case"img":z.src&&(a.src=z.src)}}break;case 6:break;case 4:break;case 12:break;case 13:if(i.memoizedState===null){var ae=i.alternate;if(ae!==null){var ye=ae.memoizedState;if(ye!==null){var we=ye.dehydrated;we!==null&&va(we)}}}break;case 19:case 17:case 21:case 22:case 23:case 25:break;default:throw Error(t(163))}hn||i.flags&512&&bc(i)}catch(_e){Wt(i,i.return,_e)}}if(i===n){We=null;break}if(a=i.sibling,a!==null){a.return=i.return,We=a;break}We=i.return}}function jp(n){for(;We!==null;){var i=We;if(i===n){We=null;break}var a=i.sibling;if(a!==null){a.return=i.return,We=a;break}We=i.return}}function Xp(n){for(;We!==null;){var i=We;try{switch(i.tag){case 0:case 11:case 15:var a=i.return;try{Ko(4,i)}catch(z){Wt(i,a,z)}break;case 1:var l=i.stateNode;if(typeof l.componentDidMount=="function"){var d=i.return;try{l.componentDidMount()}catch(z){Wt(i,d,z)}}var p=i.return;try{bc(i)}catch(z){Wt(i,p,z)}break;case 5:var E=i.return;try{bc(i)}catch(z){Wt(i,E,z)}}}catch(z){Wt(i,i.return,z)}if(i===n){We=null;break}var D=i.sibling;if(D!==null){D.return=i.return,We=D;break}We=i.return}}var Uv=Math.ceil,Zo=L.ReactCurrentDispatcher,Nc=L.ReactCurrentOwner,Xn=L.ReactCurrentBatchConfig,Mt=0,sn=null,Xt=null,un=0,On=0,ws=er(0),Kt=0,Ha=null,Ir=0,Qo=0,Dc=0,Va=null,Rn=null,Ic=0,Ts=1/0,Ni=null,Jo=!1,Uc=null,ar=null,el=!1,or=null,tl=0,Ga=0,Fc=null,nl=-1,il=0;function vn(){return(Mt&6)!==0?Le():nl!==-1?nl:nl=Le()}function lr(n){return(n.mode&1)===0?1:(Mt&2)!==0&&un!==0?un&-un:vv.transition!==null?(il===0&&(il=fo()),il):(n=Pt,n!==0||(n=window.event,n=n===void 0?16:qd(n.type)),n)}function ri(n,i,a,l){if(50<Ga)throw Ga=0,Fc=null,Error(t(185));ha(n,a,l),((Mt&2)===0||n!==sn)&&(n===sn&&((Mt&2)===0&&(Qo|=a),Kt===4&&ur(n,un)),bn(n,l),a===1&&Mt===0&&(i.mode&1)===0&&(Ts=Le()+500,Lo&&nr()))}function bn(n,i){var a=n.callbackNode;wn(n,i);var l=Dn(n,n===sn?un:0);if(l===0)a!==null&&Pe(a),n.callbackNode=null,n.callbackPriority=0;else if(i=l&-l,n.callbackPriority!==i){if(a!=null&&Pe(a),i===1)n.tag===0?_v(qp.bind(null,n)):Dh(qp.bind(null,n)),hv(function(){(Mt&6)===0&&nr()}),a=null;else{switch(zd(l)){case 1:a=st;break;case 4:a=et;break;case 16:a=St;break;case 536870912:a=It;break;default:a=St}a=nm(a,Yp.bind(null,n))}n.callbackPriority=i,n.callbackNode=a}}function Yp(n,i){if(nl=-1,il=0,(Mt&6)!==0)throw Error(t(327));var a=n.callbackNode;if(As()&&n.callbackNode!==a)return null;var l=Dn(n,n===sn?un:0);if(l===0)return null;if((l&30)!==0||(l&n.expiredLanes)!==0||i)i=rl(n,l);else{i=l;var d=Mt;Mt|=2;var p=Kp();(sn!==n||un!==i)&&(Ni=null,Ts=Le()+500,Fr(n,i));do try{kv();break}catch(D){$p(n,D)}while(!0);ec(),Zo.current=p,Mt=d,Xt!==null?i=0:(sn=null,un=0,i=Kt)}if(i!==0){if(i===2&&(d=Ar(n),d!==0&&(l=d,i=Oc(n,d))),i===1)throw a=Ha,Fr(n,0),ur(n,l),bn(n,Le()),a;if(i===6)ur(n,l);else{if(d=n.current.alternate,(l&30)===0&&!Fv(d)&&(i=rl(n,l),i===2&&(p=Ar(n),p!==0&&(l=p,i=Oc(n,p))),i===1))throw a=Ha,Fr(n,0),ur(n,l),bn(n,Le()),a;switch(n.finishedWork=d,n.finishedLanes=l,i){case 0:case 1:throw Error(t(345));case 2:Or(n,Rn,Ni);break;case 3:if(ur(n,l),(l&130023424)===l&&(i=Ic+500-Le(),10<i)){if(Dn(n,0)!==0)break;if(d=n.suspendedLanes,(d&l)!==l){vn(),n.pingedLanes|=n.suspendedLanes&d;break}n.timeoutHandle=Gu(Or.bind(null,n,Rn,Ni),i);break}Or(n,Rn,Ni);break;case 4:if(ur(n,l),(l&4194240)===l)break;for(i=n.eventTimes,d=-1;0<l;){var E=31-bt(l);p=1<<E,E=i[E],E>d&&(d=E),l&=~p}if(l=d,l=Le()-l,l=(120>l?120:480>l?480:1080>l?1080:1920>l?1920:3e3>l?3e3:4320>l?4320:1960*Uv(l/1960))-l,10<l){n.timeoutHandle=Gu(Or.bind(null,n,Rn,Ni),l);break}Or(n,Rn,Ni);break;case 5:Or(n,Rn,Ni);break;default:throw Error(t(329))}}}return bn(n,Le()),n.callbackNode===a?Yp.bind(null,n):null}function Oc(n,i){var a=Va;return n.current.memoizedState.isDehydrated&&(Fr(n,i).flags|=256),n=rl(n,i),n!==2&&(i=Rn,Rn=a,i!==null&&kc(i)),n}function kc(n){Rn===null?Rn=n:Rn.push.apply(Rn,n)}function Fv(n){for(var i=n;;){if(i.flags&16384){var a=i.updateQueue;if(a!==null&&(a=a.stores,a!==null))for(var l=0;l<a.length;l++){var d=a[l],p=d.getSnapshot;d=d.value;try{if(!Jn(p(),d))return!1}catch{return!1}}}if(a=i.child,i.subtreeFlags&16384&&a!==null)a.return=i,i=a;else{if(i===n)break;for(;i.sibling===null;){if(i.return===null||i.return===n)return!0;i=i.return}i.sibling.return=i.return,i=i.sibling}}return!0}function ur(n,i){for(i&=~Dc,i&=~Qo,n.suspendedLanes|=i,n.pingedLanes&=~i,n=n.expirationTimes;0<i;){var a=31-bt(i),l=1<<a;n[a]=-1,i&=~l}}function qp(n){if((Mt&6)!==0)throw Error(t(327));As();var i=Dn(n,0);if((i&1)===0)return bn(n,Le()),null;var a=rl(n,i);if(n.tag!==0&&a===2){var l=Ar(n);l!==0&&(i=l,a=Oc(n,l))}if(a===1)throw a=Ha,Fr(n,0),ur(n,i),bn(n,Le()),a;if(a===6)throw Error(t(345));return n.finishedWork=n.current.alternate,n.finishedLanes=i,Or(n,Rn,Ni),bn(n,Le()),null}function Bc(n,i){var a=Mt;Mt|=1;try{return n(i)}finally{Mt=a,Mt===0&&(Ts=Le()+500,Lo&&nr())}}function Ur(n){or!==null&&or.tag===0&&(Mt&6)===0&&As();var i=Mt;Mt|=1;var a=Xn.transition,l=Pt;try{if(Xn.transition=null,Pt=1,n)return n()}finally{Pt=l,Xn.transition=a,Mt=i,(Mt&6)===0&&nr()}}function zc(){On=ws.current,Ft(ws)}function Fr(n,i){n.finishedWork=null,n.finishedLanes=0;var a=n.timeoutHandle;if(a!==-1&&(n.timeoutHandle=-1,dv(a)),Xt!==null)for(a=Xt.return;a!==null;){var l=a;switch($u(l),l.tag){case 1:l=l.type.childContextTypes,l!=null&&bo();break;case 3:Ss(),Ft(Tn),Ft(cn),lc();break;case 5:ac(l);break;case 4:Ss();break;case 13:Ft(Ht);break;case 19:Ft(Ht);break;case 10:tc(l.type._context);break;case 22:case 23:zc()}a=a.return}if(sn=n,Xt=n=cr(n.current,null),un=On=i,Kt=0,Ha=null,Dc=Qo=Ir=0,Rn=Va=null,Lr!==null){for(i=0;i<Lr.length;i++)if(a=Lr[i],l=a.interleaved,l!==null){a.interleaved=null;var d=l.next,p=a.pending;if(p!==null){var E=p.next;p.next=d,l.next=E}a.pending=l}Lr=null}return n}function $p(n,i){do{var a=Xt;try{if(ec(),Ho.current=jo,Vo){for(var l=Vt.memoizedState;l!==null;){var d=l.queue;d!==null&&(d.pending=null),l=l.next}Vo=!1}if(Dr=0,rn=$t=Vt=null,Ua=!1,Fa=0,Nc.current=null,a===null||a.return===null){Kt=1,Ha=i,Xt=null;break}e:{var p=n,E=a.return,D=a,z=i;if(i=un,D.flags|=32768,z!==null&&typeof z=="object"&&typeof z.then=="function"){var ae=z,ye=D,we=ye.tag;if((ye.mode&1)===0&&(we===0||we===11||we===15)){var _e=ye.alternate;_e?(ye.updateQueue=_e.updateQueue,ye.memoizedState=_e.memoizedState,ye.lanes=_e.lanes):(ye.updateQueue=null,ye.memoizedState=null)}var He=yp(E);if(He!==null){He.flags&=-257,Sp(He,E,D,p,i),He.mode&1&&xp(p,ae,i),i=He,z=ae;var je=i.updateQueue;if(je===null){var Ye=new Set;Ye.add(z),i.updateQueue=Ye}else je.add(z);break e}else{if((i&1)===0){xp(p,ae,i),Hc();break e}z=Error(t(426))}}else if(zt&&D.mode&1){var jt=yp(E);if(jt!==null){(jt.flags&65536)===0&&(jt.flags|=256),Sp(jt,E,D,p,i),Qu(Ms(z,D));break e}}p=z=Ms(z,D),Kt!==4&&(Kt=2),Va===null?Va=[p]:Va.push(p),p=E;do{switch(p.tag){case 3:p.flags|=65536,i&=-i,p.lanes|=i;var $=_p(p,z,i);Wh(p,$);break e;case 1:D=z;var W=p.type,ee=p.stateNode;if((p.flags&128)===0&&(typeof W.getDerivedStateFromError=="function"||ee!==null&&typeof ee.componentDidCatch=="function"&&(ar===null||!ar.has(ee)))){p.flags|=65536,i&=-i,p.lanes|=i;var be=vp(p,D,i);Wh(p,be);break e}}p=p.return}while(p!==null)}Qp(a)}catch(qe){i=qe,Xt===a&&a!==null&&(Xt=a=a.return);continue}break}while(!0)}function Kp(){var n=Zo.current;return Zo.current=jo,n===null?jo:n}function Hc(){(Kt===0||Kt===3||Kt===2)&&(Kt=4),sn===null||(Ir&268435455)===0&&(Qo&268435455)===0||ur(sn,un)}function rl(n,i){var a=Mt;Mt|=2;var l=Kp();(sn!==n||un!==i)&&(Ni=null,Fr(n,i));do try{Ov();break}catch(d){$p(n,d)}while(!0);if(ec(),Mt=a,Zo.current=l,Xt!==null)throw Error(t(261));return sn=null,un=0,Kt}function Ov(){for(;Xt!==null;)Zp(Xt)}function kv(){for(;Xt!==null&&!ze();)Zp(Xt)}function Zp(n){var i=tm(n.alternate,n,On);n.memoizedProps=n.pendingProps,i===null?Qp(n):Xt=i,Nc.current=null}function Qp(n){var i=n;do{var a=i.alternate;if(n=i.return,(i.flags&32768)===0){if(a=Pv(a,i,On),a!==null){Xt=a;return}}else{if(a=Lv(a,i),a!==null){a.flags&=32767,Xt=a;return}if(n!==null)n.flags|=32768,n.subtreeFlags=0,n.deletions=null;else{Kt=6,Xt=null;return}}if(i=i.sibling,i!==null){Xt=i;return}Xt=i=n}while(i!==null);Kt===0&&(Kt=5)}function Or(n,i,a){var l=Pt,d=Xn.transition;try{Xn.transition=null,Pt=1,Bv(n,i,a,l)}finally{Xn.transition=d,Pt=l}return null}function Bv(n,i,a,l){do As();while(or!==null);if((Mt&6)!==0)throw Error(t(327));a=n.finishedWork;var d=n.finishedLanes;if(a===null)return null;if(n.finishedWork=null,n.finishedLanes=0,a===n.current)throw Error(t(177));n.callbackNode=null,n.callbackPriority=0;var p=a.lanes|a.childLanes;if(x_(n,p),n===sn&&(Xt=sn=null,un=0),(a.subtreeFlags&2064)===0&&(a.flags&2064)===0||el||(el=!0,nm(St,function(){return As(),null})),p=(a.flags&15990)!==0,(a.subtreeFlags&15990)!==0||p){p=Xn.transition,Xn.transition=null;var E=Pt;Pt=1;var D=Mt;Mt|=4,Nc.current=null,Dv(n,a),Vp(a,n),sv(Hu),mo=!!zu,Hu=zu=null,n.current=a,Iv(a),Xe(),Mt=D,Pt=E,Xn.transition=p}else n.current=a;if(el&&(el=!1,or=n,tl=d),p=n.pendingLanes,p===0&&(ar=null),$e(a.stateNode),bn(n,Le()),i!==null)for(l=n.onRecoverableError,a=0;a<i.length;a++)d=i[a],l(d.value,{componentStack:d.stack,digest:d.digest});if(Jo)throw Jo=!1,n=Uc,Uc=null,n;return(tl&1)!==0&&n.tag!==0&&As(),p=n.pendingLanes,(p&1)!==0?n===Fc?Ga++:(Ga=0,Fc=n):Ga=0,nr(),null}function As(){if(or!==null){var n=zd(tl),i=Xn.transition,a=Pt;try{if(Xn.transition=null,Pt=16>n?16:n,or===null)var l=!1;else{if(n=or,or=null,tl=0,(Mt&6)!==0)throw Error(t(331));var d=Mt;for(Mt|=4,We=n.current;We!==null;){var p=We,E=p.child;if((We.flags&16)!==0){var D=p.deletions;if(D!==null){for(var z=0;z<D.length;z++){var ae=D[z];for(We=ae;We!==null;){var ye=We;switch(ye.tag){case 0:case 11:case 15:za(8,ye,p)}var we=ye.child;if(we!==null)we.return=ye,We=we;else for(;We!==null;){ye=We;var _e=ye.sibling,He=ye.return;if(Op(ye),ye===ae){We=null;break}if(_e!==null){_e.return=He,We=_e;break}We=He}}}var je=p.alternate;if(je!==null){var Ye=je.child;if(Ye!==null){je.child=null;do{var jt=Ye.sibling;Ye.sibling=null,Ye=jt}while(Ye!==null)}}We=p}}if((p.subtreeFlags&2064)!==0&&E!==null)E.return=p,We=E;else e:for(;We!==null;){if(p=We,(p.flags&2048)!==0)switch(p.tag){case 0:case 11:case 15:za(9,p,p.return)}var $=p.sibling;if($!==null){$.return=p.return,We=$;break e}We=p.return}}var W=n.current;for(We=W;We!==null;){E=We;var ee=E.child;if((E.subtreeFlags&2064)!==0&&ee!==null)ee.return=E,We=ee;else e:for(E=W;We!==null;){if(D=We,(D.flags&2048)!==0)try{switch(D.tag){case 0:case 11:case 15:Ko(9,D)}}catch(qe){Wt(D,D.return,qe)}if(D===E){We=null;break e}var be=D.sibling;if(be!==null){be.return=D.return,We=be;break e}We=D.return}}if(Mt=d,nr(),dt&&typeof dt.onPostCommitFiberRoot=="function")try{dt.onPostCommitFiberRoot(tn,n)}catch{}l=!0}return l}finally{Pt=a,Xn.transition=i}}return!1}function Jp(n,i,a){i=Ms(a,i),i=_p(n,i,1),n=rr(n,i,1),i=vn(),n!==null&&(ha(n,1,i),bn(n,i))}function Wt(n,i,a){if(n.tag===3)Jp(n,n,a);else for(;i!==null;){if(i.tag===3){Jp(i,n,a);break}else if(i.tag===1){var l=i.stateNode;if(typeof i.type.getDerivedStateFromError=="function"||typeof l.componentDidCatch=="function"&&(ar===null||!ar.has(l))){n=Ms(a,n),n=vp(i,n,1),i=rr(i,n,1),n=vn(),i!==null&&(ha(i,1,n),bn(i,n));break}}i=i.return}}function zv(n,i,a){var l=n.pingCache;l!==null&&l.delete(i),i=vn(),n.pingedLanes|=n.suspendedLanes&a,sn===n&&(un&a)===a&&(Kt===4||Kt===3&&(un&130023424)===un&&500>Le()-Ic?Fr(n,0):Dc|=a),bn(n,i)}function em(n,i){i===0&&((n.mode&1)===0?i=1:(i=Xi,Xi<<=1,(Xi&130023424)===0&&(Xi=4194304)));var a=vn();n=bi(n,i),n!==null&&(ha(n,i,a),bn(n,a))}function Hv(n){var i=n.memoizedState,a=0;i!==null&&(a=i.retryLane),em(n,a)}function Vv(n,i){var a=0;switch(n.tag){case 13:var l=n.stateNode,d=n.memoizedState;d!==null&&(a=d.retryLane);break;case 19:l=n.stateNode;break;default:throw Error(t(314))}l!==null&&l.delete(i),em(n,a)}var tm;tm=function(n,i,a){if(n!==null)if(n.memoizedProps!==i.pendingProps||Tn.current)Cn=!0;else{if((n.lanes&a)===0&&(i.flags&128)===0)return Cn=!1,bv(n,i,a);Cn=(n.flags&131072)!==0}else Cn=!1,zt&&(i.flags&1048576)!==0&&Ih(i,Do,i.index);switch(i.lanes=0,i.tag){case 2:var l=i.type;qo(n,i),n=i.pendingProps;var d=ps(i,cn.current);ys(i,a),d=fc(null,i,l,n,d,a);var p=dc();return i.flags|=1,typeof d=="object"&&d!==null&&typeof d.render=="function"&&d.$$typeof===void 0?(i.tag=1,i.memoizedState=null,i.updateQueue=null,An(l)?(p=!0,Po(i)):p=!1,i.memoizedState=d.state!==null&&d.state!==void 0?d.state:null,rc(i),d.updater=Xo,i.stateNode=d,d._reactInternals=i,vc(i,l,n,a),i=Mc(null,i,l,!0,p,a)):(i.tag=0,zt&&p&&qu(i),_n(null,i,d,a),i=i.child),i;case 16:l=i.elementType;e:{switch(qo(n,i),n=i.pendingProps,d=l._init,l=d(l._payload),i.type=l,d=i.tag=Wv(l),n=ti(l,n),d){case 0:i=Sc(null,i,l,n,a);break e;case 1:i=Cp(null,i,l,n,a);break e;case 11:i=Mp(null,i,l,n,a);break e;case 14:i=Ep(null,i,l,ti(l.type,n),a);break e}throw Error(t(306,l,""))}return i;case 0:return l=i.type,d=i.pendingProps,d=i.elementType===l?d:ti(l,d),Sc(n,i,l,d,a);case 1:return l=i.type,d=i.pendingProps,d=i.elementType===l?d:ti(l,d),Cp(n,i,l,d,a);case 3:e:{if(Rp(i),n===null)throw Error(t(387));l=i.pendingProps,p=i.memoizedState,d=p.element,Gh(n,i),Bo(i,l,null,a);var E=i.memoizedState;if(l=E.element,p.isDehydrated)if(p={element:l,isDehydrated:!1,cache:E.cache,pendingSuspenseBoundaries:E.pendingSuspenseBoundaries,transitions:E.transitions},i.updateQueue.baseState=p,i.memoizedState=p,i.flags&256){d=Ms(Error(t(423)),i),i=bp(n,i,l,a,d);break e}else if(l!==d){d=Ms(Error(t(424)),i),i=bp(n,i,l,a,d);break e}else for(Fn=Ji(i.stateNode.containerInfo.firstChild),Un=i,zt=!0,ei=null,a=Hh(i,null,l,a),i.child=a;a;)a.flags=a.flags&-3|4096,a=a.sibling;else{if(_s(),l===d){i=Li(n,i,a);break e}_n(n,i,l,a)}i=i.child}return i;case 5:return Xh(i),n===null&&Zu(i),l=i.type,d=i.pendingProps,p=n!==null?n.memoizedProps:null,E=d.children,Vu(l,d)?E=null:p!==null&&Vu(l,p)&&(i.flags|=32),Ap(n,i),_n(n,i,E,a),i.child;case 6:return n===null&&Zu(i),null;case 13:return Pp(n,i,a);case 4:return sc(i,i.stateNode.containerInfo),l=i.pendingProps,n===null?i.child=vs(i,null,l,a):_n(n,i,l,a),i.child;case 11:return l=i.type,d=i.pendingProps,d=i.elementType===l?d:ti(l,d),Mp(n,i,l,d,a);case 7:return _n(n,i,i.pendingProps,a),i.child;case 8:return _n(n,i,i.pendingProps.children,a),i.child;case 12:return _n(n,i,i.pendingProps.children,a),i.child;case 10:e:{if(l=i.type._context,d=i.pendingProps,p=i.memoizedProps,E=d.value,Dt(Fo,l._currentValue),l._currentValue=E,p!==null)if(Jn(p.value,E)){if(p.children===d.children&&!Tn.current){i=Li(n,i,a);break e}}else for(p=i.child,p!==null&&(p.return=i);p!==null;){var D=p.dependencies;if(D!==null){E=p.child;for(var z=D.firstContext;z!==null;){if(z.context===l){if(p.tag===1){z=Pi(-1,a&-a),z.tag=2;var ae=p.updateQueue;if(ae!==null){ae=ae.shared;var ye=ae.pending;ye===null?z.next=z:(z.next=ye.next,ye.next=z),ae.pending=z}}p.lanes|=a,z=p.alternate,z!==null&&(z.lanes|=a),nc(p.return,a,i),D.lanes|=a;break}z=z.next}}else if(p.tag===10)E=p.type===i.type?null:p.child;else if(p.tag===18){if(E=p.return,E===null)throw Error(t(341));E.lanes|=a,D=E.alternate,D!==null&&(D.lanes|=a),nc(E,a,i),E=p.sibling}else E=p.child;if(E!==null)E.return=p;else for(E=p;E!==null;){if(E===i){E=null;break}if(p=E.sibling,p!==null){p.return=E.return,E=p;break}E=E.return}p=E}_n(n,i,d.children,a),i=i.child}return i;case 9:return d=i.type,l=i.pendingProps.children,ys(i,a),d=Wn(d),l=l(d),i.flags|=1,_n(n,i,l,a),i.child;case 14:return l=i.type,d=ti(l,i.pendingProps),d=ti(l.type,d),Ep(n,i,l,d,a);case 15:return wp(n,i,i.type,i.pendingProps,a);case 17:return l=i.type,d=i.pendingProps,d=i.elementType===l?d:ti(l,d),qo(n,i),i.tag=1,An(l)?(n=!0,Po(i)):n=!1,ys(i,a),mp(i,l,d),vc(i,l,d,a),Mc(null,i,l,!0,n,a);case 19:return Np(n,i,a);case 22:return Tp(n,i,a)}throw Error(t(156,i.tag))};function nm(n,i){return q(n,i)}function Gv(n,i,a,l){this.tag=n,this.key=a,this.sibling=this.child=this.return=this.stateNode=this.type=this.elementType=null,this.index=0,this.ref=null,this.pendingProps=i,this.dependencies=this.memoizedState=this.updateQueue=this.memoizedProps=null,this.mode=l,this.subtreeFlags=this.flags=0,this.deletions=null,this.childLanes=this.lanes=0,this.alternate=null}function Yn(n,i,a,l){return new Gv(n,i,a,l)}function Vc(n){return n=n.prototype,!(!n||!n.isReactComponent)}function Wv(n){if(typeof n=="function")return Vc(n)?1:0;if(n!=null){if(n=n.$$typeof,n===K)return 11;if(n===te)return 14}return 2}function cr(n,i){var a=n.alternate;return a===null?(a=Yn(n.tag,i,n.key,n.mode),a.elementType=n.elementType,a.type=n.type,a.stateNode=n.stateNode,a.alternate=n,n.alternate=a):(a.pendingProps=i,a.type=n.type,a.flags=0,a.subtreeFlags=0,a.deletions=null),a.flags=n.flags&14680064,a.childLanes=n.childLanes,a.lanes=n.lanes,a.child=n.child,a.memoizedProps=n.memoizedProps,a.memoizedState=n.memoizedState,a.updateQueue=n.updateQueue,i=n.dependencies,a.dependencies=i===null?null:{lanes:i.lanes,firstContext:i.firstContext},a.sibling=n.sibling,a.index=n.index,a.ref=n.ref,a}function sl(n,i,a,l,d,p){var E=2;if(l=n,typeof n=="function")Vc(n)&&(E=1);else if(typeof n=="string")E=5;else e:switch(n){case N:return kr(a.children,d,p,i);case H:E=8,d|=8;break;case b:return n=Yn(12,a,i,d|2),n.elementType=b,n.lanes=p,n;case V:return n=Yn(13,a,i,d),n.elementType=V,n.lanes=p,n;case Z:return n=Yn(19,a,i,d),n.elementType=Z,n.lanes=p,n;case se:return al(a,d,p,i);default:if(typeof n=="object"&&n!==null)switch(n.$$typeof){case w:E=10;break e;case I:E=9;break e;case K:E=11;break e;case te:E=14;break e;case ue:E=16,l=null;break e}throw Error(t(130,n==null?n:typeof n,""))}return i=Yn(E,a,i,d),i.elementType=n,i.type=l,i.lanes=p,i}function kr(n,i,a,l){return n=Yn(7,n,l,i),n.lanes=a,n}function al(n,i,a,l){return n=Yn(22,n,l,i),n.elementType=se,n.lanes=a,n.stateNode={isHidden:!1},n}function Gc(n,i,a){return n=Yn(6,n,null,i),n.lanes=a,n}function Wc(n,i,a){return i=Yn(4,n.children!==null?n.children:[],n.key,i),i.lanes=a,i.stateNode={containerInfo:n.containerInfo,pendingChildren:null,implementation:n.implementation},i}function jv(n,i,a,l,d){this.tag=i,this.containerInfo=n,this.finishedWork=this.pingCache=this.current=this.pendingChildren=null,this.timeoutHandle=-1,this.callbackNode=this.pendingContext=this.context=null,this.callbackPriority=0,this.eventTimes=rs(0),this.expirationTimes=rs(-1),this.entangledLanes=this.finishedLanes=this.mutableReadLanes=this.expiredLanes=this.pingedLanes=this.suspendedLanes=this.pendingLanes=0,this.entanglements=rs(0),this.identifierPrefix=l,this.onRecoverableError=d,this.mutableSourceEagerHydrationData=null}function jc(n,i,a,l,d,p,E,D,z){return n=new jv(n,i,a,D,z),i===1?(i=1,p===!0&&(i|=8)):i=0,p=Yn(3,null,null,i),n.current=p,p.stateNode=n,p.memoizedState={element:l,isDehydrated:a,cache:null,transitions:null,pendingSuspenseBoundaries:null},rc(p),n}function Xv(n,i,a){var l=3<arguments.length&&arguments[3]!==void 0?arguments[3]:null;return{$$typeof:U,key:l==null?null:""+l,children:n,containerInfo:i,implementation:a}}function im(n){if(!n)return tr;n=n._reactInternals;e:{if(wi(n)!==n||n.tag!==1)throw Error(t(170));var i=n;do{switch(i.tag){case 3:i=i.stateNode.context;break e;case 1:if(An(i.type)){i=i.stateNode.__reactInternalMemoizedMergedChildContext;break e}}i=i.return}while(i!==null);throw Error(t(171))}if(n.tag===1){var a=n.type;if(An(a))return Lh(n,a,i)}return i}function rm(n,i,a,l,d,p,E,D,z){return n=jc(a,l,!0,n,d,p,E,D,z),n.context=im(null),a=n.current,l=vn(),d=lr(a),p=Pi(l,d),p.callback=i??null,rr(a,p,d),n.current.lanes=d,ha(n,d,l),bn(n,l),n}function ol(n,i,a,l){var d=i.current,p=vn(),E=lr(d);return a=im(a),i.context===null?i.context=a:i.pendingContext=a,i=Pi(p,E),i.payload={element:n},l=l===void 0?null:l,l!==null&&(i.callback=l),n=rr(d,i,E),n!==null&&(ri(n,d,E,p),ko(n,d,E)),E}function ll(n){if(n=n.current,!n.child)return null;switch(n.child.tag){case 5:return n.child.stateNode;default:return n.child.stateNode}}function sm(n,i){if(n=n.memoizedState,n!==null&&n.dehydrated!==null){var a=n.retryLane;n.retryLane=a!==0&&a<i?a:i}}function Xc(n,i){sm(n,i),(n=n.alternate)&&sm(n,i)}function Yv(){return null}var am=typeof reportError=="function"?reportError:function(n){console.error(n)};function Yc(n){this._internalRoot=n}ul.prototype.render=Yc.prototype.render=function(n){var i=this._internalRoot;if(i===null)throw Error(t(409));ol(n,i,null,null)},ul.prototype.unmount=Yc.prototype.unmount=function(){var n=this._internalRoot;if(n!==null){this._internalRoot=null;var i=n.containerInfo;Ur(function(){ol(null,n,null,null)}),i[Ti]=null}};function ul(n){this._internalRoot=n}ul.prototype.unstable_scheduleHydration=function(n){if(n){var i=Gd();n={blockedOn:null,target:n,priority:i};for(var a=0;a<Ki.length&&i!==0&&i<Ki[a].priority;a++);Ki.splice(a,0,n),a===0&&Xd(n)}};function qc(n){return!(!n||n.nodeType!==1&&n.nodeType!==9&&n.nodeType!==11)}function cl(n){return!(!n||n.nodeType!==1&&n.nodeType!==9&&n.nodeType!==11&&(n.nodeType!==8||n.nodeValue!==" react-mount-point-unstable "))}function om(){}function qv(n,i,a,l,d){if(d){if(typeof l=="function"){var p=l;l=function(){var ae=ll(E);p.call(ae)}}var E=rm(i,l,n,0,null,!1,!1,"",om);return n._reactRootContainer=E,n[Ti]=E.current,Ca(n.nodeType===8?n.parentNode:n),Ur(),E}for(;d=n.lastChild;)n.removeChild(d);if(typeof l=="function"){var D=l;l=function(){var ae=ll(z);D.call(ae)}}var z=jc(n,0,!1,null,null,!1,!1,"",om);return n._reactRootContainer=z,n[Ti]=z.current,Ca(n.nodeType===8?n.parentNode:n),Ur(function(){ol(i,z,a,l)}),z}function fl(n,i,a,l,d){var p=a._reactRootContainer;if(p){var E=p;if(typeof d=="function"){var D=d;d=function(){var z=ll(E);D.call(z)}}ol(i,E,n,d)}else E=qv(a,i,n,d,l);return ll(E)}Hd=function(n){switch(n.tag){case 3:var i=n.stateNode;if(i.current.memoizedState.isDehydrated){var a=Nt(i.pendingLanes);a!==0&&(vu(i,a|1),bn(i,Le()),(Mt&6)===0&&(Ts=Le()+500,nr()))}break;case 13:Ur(function(){var l=bi(n,1);if(l!==null){var d=vn();ri(l,n,1,d)}}),Xc(n,1)}},xu=function(n){if(n.tag===13){var i=bi(n,134217728);if(i!==null){var a=vn();ri(i,n,134217728,a)}Xc(n,134217728)}},Vd=function(n){if(n.tag===13){var i=lr(n),a=bi(n,i);if(a!==null){var l=vn();ri(a,n,i,l)}Xc(n,i)}},Gd=function(){return Pt},Wd=function(n,i){var a=Pt;try{return Pt=n,i()}finally{Pt=a}},pe=function(n,i,a){switch(i){case"input":if(De(n,a),i=a.name,a.type==="radio"&&i!=null){for(a=n;a.parentNode;)a=a.parentNode;for(a=a.querySelectorAll("input[name="+JSON.stringify(""+i)+'][type="radio"]'),i=0;i<a.length;i++){var l=a[i];if(l!==n&&l.form===n.form){var d=Ro(l);if(!d)throw Error(t(90));B(l),De(l,d)}}}break;case"textarea":Ee(n,a);break;case"select":i=a.value,i!=null&&A(n,!!a.multiple,i,!1)}},at=Bc,tt=Ur;var $v={usingClientEntryPoint:!1,Events:[Pa,ds,Ro,vt,xe,Bc]},Wa={findFiberByHostInstance:Cr,bundleType:0,version:"18.3.1",rendererPackageName:"react-dom"},Kv={bundleType:Wa.bundleType,version:Wa.version,rendererPackageName:Wa.rendererPackageName,rendererConfig:Wa.rendererConfig,overrideHookState:null,overrideHookStateDeletePath:null,overrideHookStateRenamePath:null,overrideProps:null,overridePropsDeletePath:null,overridePropsRenamePath:null,setErrorHandler:null,setSuspenseHandler:null,scheduleUpdate:null,currentDispatcherRef:L.ReactCurrentDispatcher,findHostInstanceByFiber:function(n){return n=re(n),n===null?null:n.stateNode},findFiberByHostInstance:Wa.findFiberByHostInstance||Yv,findHostInstancesForRefresh:null,scheduleRefresh:null,scheduleRoot:null,setRefreshHandler:null,getCurrentFiber:null,reconcilerVersion:"18.3.1-next-f1338f8080-20240426"};if(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__<"u"){var dl=__REACT_DEVTOOLS_GLOBAL_HOOK__;if(!dl.isDisabled&&dl.supportsFiber)try{tn=dl.inject(Kv),dt=dl}catch{}}return Pn.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=$v,Pn.createPortal=function(n,i){var a=2<arguments.length&&arguments[2]!==void 0?arguments[2]:null;if(!qc(i))throw Error(t(200));return Xv(n,i,null,a)},Pn.createRoot=function(n,i){if(!qc(n))throw Error(t(299));var a=!1,l="",d=am;return i!=null&&(i.unstable_strictMode===!0&&(a=!0),i.identifierPrefix!==void 0&&(l=i.identifierPrefix),i.onRecoverableError!==void 0&&(d=i.onRecoverableError)),i=jc(n,1,!1,null,null,a,!1,l,d),n[Ti]=i.current,Ca(n.nodeType===8?n.parentNode:n),new Yc(i)},Pn.findDOMNode=function(n){if(n==null)return null;if(n.nodeType===1)return n;var i=n._reactInternals;if(i===void 0)throw typeof n.render=="function"?Error(t(188)):(n=Object.keys(n).join(","),Error(t(268,n)));return n=re(i),n=n===null?null:n.stateNode,n},Pn.flushSync=function(n){return Ur(n)},Pn.hydrate=function(n,i,a){if(!cl(i))throw Error(t(200));return fl(null,n,i,!0,a)},Pn.hydrateRoot=function(n,i,a){if(!qc(n))throw Error(t(405));var l=a!=null&&a.hydratedSources||null,d=!1,p="",E=am;if(a!=null&&(a.unstable_strictMode===!0&&(d=!0),a.identifierPrefix!==void 0&&(p=a.identifierPrefix),a.onRecoverableError!==void 0&&(E=a.onRecoverableError)),i=rm(i,null,n,1,a??null,d,!1,p,E),n[Ti]=i.current,Ca(n),l)for(n=0;n<l.length;n++)a=l[n],d=a._getVersion,d=d(a._source),i.mutableSourceEagerHydrationData==null?i.mutableSourceEagerHydrationData=[a,d]:i.mutableSourceEagerHydrationData.push(a,d);return new ul(i)},Pn.render=function(n,i,a){if(!cl(i))throw Error(t(200));return fl(null,n,i,!1,a)},Pn.unmountComponentAtNode=function(n){if(!cl(n))throw Error(t(40));return n._reactRootContainer?(Ur(function(){fl(null,null,n,!1,function(){n._reactRootContainer=null,n[Ti]=null})}),!0):!1},Pn.unstable_batchedUpdates=Bc,Pn.unstable_renderSubtreeIntoContainer=function(n,i,a,l){if(!cl(a))throw Error(t(200));if(n==null||n._reactInternals===void 0)throw Error(t(38));return fl(n,i,a,!1,l)},Pn.version="18.3.1-next-f1338f8080-20240426",Pn}var mm;function a0(){if(mm)return Zc.exports;mm=1;function s(){if(!(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__>"u"||typeof __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE!="function"))try{__REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE(s)}catch(e){console.error(e)}}return s(),Zc.exports=s0(),Zc.exports}var gm;function o0(){if(gm)return hl;gm=1;var s=a0();return hl.createRoot=s.createRoot,hl.hydrateRoot=s.hydrateRoot,hl}var l0=o0();class _m{constructor(){this.ctx=null,this.armed=!0,this.quietAdvisories=!1,this._mode="off",this._osc=null,this._gain=null,this._lfo=null,this._timer=null,this._lastPubBeep=0,this._status={leak:!1,publisher:"never",stream:"connecting"},this._raf=null}async arm(){if(!this.ctx){const e=window.AudioContext||window.webkitAudioContext;this.ctx=new e}this.ctx.state==="suspended"&&await this.ctx.resume(),this.armed=!0,this._ensureLoop()}setQuietAdvisories(e){this.quietAdvisories=e,localStorage.setItem("rov.alert.quietAdvisories",e?"1":"0"),localStorage.removeItem("rov.alert.muted"),this._apply()}setMuted(e){this.setQuietAdvisories(e)}get muted(){return this.quietAdvisories}set muted(e){this.quietAdvisories=!!e}static loadPrefs(){const e=localStorage.getItem("rov.alert.muted")==="1",t=localStorage.getItem("rov.alert.quietAdvisories")==="1"||e;return{muted:t,quietAdvisories:t}}update(e){this._status={...this._status,...e},this._ensureLoop(),this._apply()}_ensureLoop(){if(this._raf!=null)return;const e=()=>{this._raf=window.setTimeout(e,500),this._apply()};this._raf=window.setTimeout(e,500)}_apply(){if(!this.armed||!this.ctx){this._stop();return}this.ctx.state==="suspended"&&this.ctx.resume().catch(()=>{});const{leak:e,publisher:t,stream:r}=this._status;if(e){this._siren();return}if(this.quietAdvisories){this._stop();return}if(r==="disconnected"){this._pulse(180,5);return}if(t==="stale"||t==="never"){const o=performance.now()/1e3;o-this._lastPubBeep>20&&(this._lastPubBeep=o,this._beep(420,.2)),(this._mode==="siren"||this._mode==="pulse")&&this._stop();return}this._stop()}_ensureVoice(){if(this._osc)return;const e=this.ctx;this._osc=e.createOscillator(),this._gain=e.createGain(),this._osc.type="square",this._gain.gain.value=0,this._osc.connect(this._gain),this._gain.connect(e.destination),this._osc.start()}_siren(){if(this._ensureVoice(),this._mode==="siren")return;this._stopLfo(),this._mode="siren",this._osc.type="square";const e=this.ctx.currentTime;this._gain.gain.cancelScheduledValues(e),this._gain.gain.setValueAtTime(0,e),this._gain.gain.linearRampToValueAtTime(.12,e+.05),this._lfo=this.ctx.createOscillator();const t=this.ctx.createGain();this._lfo.frequency.value=4,t.gain.value=180,this._osc.frequency.value=620,this._lfo.connect(t),t.connect(this._osc.frequency),this._lfo.start(),this._lfoGain=t}_pulse(e,t){if(this._ensureVoice(),this._mode==="pulse")return;this._stopLfo(),this._mode="pulse",this._osc.type="sawtooth",this._osc.frequency.value=e;const r=()=>{if(this._mode!=="pulse"||!this._gain)return;const o=this.ctx.currentTime;this._gain.gain.cancelScheduledValues(o),this._gain.gain.setValueAtTime(0,o),this._gain.gain.linearRampToValueAtTime(.07,o+.02),this._gain.gain.linearRampToValueAtTime(0,o+.18)};r(),this._timer=window.setInterval(r,t*1e3)}_beep(e,t){if(!this.ctx)return;const r=this.ctx.createOscillator(),o=this.ctx.createGain();r.frequency.value=e,r.type="triangle",o.gain.value=.06,r.connect(o),o.connect(this.ctx.destination),r.start(),o.gain.exponentialRampToValueAtTime(.001,this.ctx.currentTime+t),r.stop(this.ctx.currentTime+t)}_stopLfo(){if(this._lfo){try{this._lfo.stop()}catch{}this._lfo.disconnect(),this._lfo=null}this._lfoGain&&(this._lfoGain.disconnect(),this._lfoGain=null),this._timer&&(clearInterval(this._timer),this._timer=null)}_stop(){this._stopLfo(),this._gain&&this._gain.gain.setTargetAtTime(0,this.ctx?this.ctx.currentTime:0,.02),this._mode="off"}}/**
 * @license
 * Copyright 2010-2024 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const Md="167",u0=0,vm=1,c0=2,Dg=1,f0=2,ki=3,Mr=0,Ln=1,Bi=2,xr=0,Js=1,xm=2,ym=3,Sm=4,d0=5,qr=100,h0=101,p0=102,m0=103,g0=104,_0=200,v0=201,x0=202,y0=203,zf=204,Hf=205,S0=206,M0=207,E0=208,w0=209,T0=210,A0=211,C0=212,R0=213,b0=214,P0=0,L0=1,N0=2,tu=3,D0=4,I0=5,U0=6,F0=7,Ig=0,O0=1,k0=2,yr=0,B0=1,z0=2,H0=3,V0=4,G0=5,W0=6,j0=7,Ug=300,sa=301,aa=302,Vf=303,Gf=304,cu=306,Wf=1e3,Kr=1001,jf=1002,Kn=1003,X0=1004,pl=1005,ci=1006,ef=1007,Zr=1008,Vi=1009,Fg=1010,Og=1011,no=1012,Ed=1013,es=1014,zi=1015,ro=1016,wd=1017,Td=1018,oa=1020,kg=35902,Bg=1021,zg=1022,di=1023,Hg=1024,Vg=1025,ea=1026,la=1027,Gg=1028,Ad=1029,Wg=1030,Cd=1031,Rd=1033,ql=33776,$l=33777,Kl=33778,Zl=33779,Xf=35840,Yf=35841,qf=35842,$f=35843,Kf=36196,Zf=37492,Qf=37496,Jf=37808,ed=37809,td=37810,nd=37811,id=37812,rd=37813,sd=37814,ad=37815,od=37816,ld=37817,ud=37818,cd=37819,fd=37820,dd=37821,Ql=36492,hd=36494,pd=36495,jg=36283,md=36284,gd=36285,_d=36286,Y0=3200,q0=3201,Xg=0,$0=1,vr="",ui="srgb",wr="srgb-linear",bd="display-p3",fu="display-p3-linear",nu="linear",Ot="srgb",iu="rec709",ru="p3",Cs=7680,Mm=519,K0=512,Z0=513,Q0=514,Yg=515,J0=516,ex=517,tx=518,nx=519,vd=35044,Em="300 es",Hi=2e3,su=2001;class ca{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});const r=this._listeners;r[e]===void 0&&(r[e]=[]),r[e].indexOf(t)===-1&&r[e].push(t)}hasEventListener(e,t){if(this._listeners===void 0)return!1;const r=this._listeners;return r[e]!==void 0&&r[e].indexOf(t)!==-1}removeEventListener(e,t){if(this._listeners===void 0)return;const o=this._listeners[e];if(o!==void 0){const u=o.indexOf(t);u!==-1&&o.splice(u,1)}}dispatchEvent(e){if(this._listeners===void 0)return;const r=this._listeners[e.type];if(r!==void 0){e.target=this;const o=r.slice(0);for(let u=0,c=o.length;u<c;u++)o[u].call(this,e);e.target=null}}}const pn=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],tf=Math.PI/180,xd=180/Math.PI;function Sr(){const s=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,r=Math.random()*4294967295|0;return(pn[s&255]+pn[s>>8&255]+pn[s>>16&255]+pn[s>>24&255]+"-"+pn[e&255]+pn[e>>8&255]+"-"+pn[e>>16&15|64]+pn[e>>24&255]+"-"+pn[t&63|128]+pn[t>>8&255]+"-"+pn[t>>16&255]+pn[t>>24&255]+pn[r&255]+pn[r>>8&255]+pn[r>>16&255]+pn[r>>24&255]).toLowerCase()}function Sn(s,e,t){return Math.max(e,Math.min(t,s))}function ix(s,e){return(s%e+e)%e}function nf(s,e,t){return(1-t)*s+t*e}function xi(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return s/4294967295;case Uint16Array:return s/65535;case Uint8Array:return s/255;case Int32Array:return Math.max(s/2147483647,-1);case Int16Array:return Math.max(s/32767,-1);case Int8Array:return Math.max(s/127,-1);default:throw new Error("Invalid component type.")}}function Lt(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return Math.round(s*4294967295);case Uint16Array:return Math.round(s*65535);case Uint8Array:return Math.round(s*255);case Int32Array:return Math.round(s*2147483647);case Int16Array:return Math.round(s*32767);case Int8Array:return Math.round(s*127);default:throw new Error("Invalid component type.")}}class ht{constructor(e=0,t=0){ht.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){const t=this.x,r=this.y,o=e.elements;return this.x=o[0]*t+o[3]*r+o[6],this.y=o[1]*t+o[4]*r+o[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const r=this.dot(e)/t;return Math.acos(Sn(r,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,r=this.y-e.y;return t*t+r*r}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){const r=Math.cos(t),o=Math.sin(t),u=this.x-e.x,c=this.y-e.y;return this.x=u*r-c*o+e.x,this.y=u*o+c*r+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class mt{constructor(e,t,r,o,u,c,f,h,m){mt.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,r,o,u,c,f,h,m)}set(e,t,r,o,u,c,f,h,m){const g=this.elements;return g[0]=e,g[1]=o,g[2]=f,g[3]=t,g[4]=u,g[5]=h,g[6]=r,g[7]=c,g[8]=m,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){const t=this.elements,r=e.elements;return t[0]=r[0],t[1]=r[1],t[2]=r[2],t[3]=r[3],t[4]=r[4],t[5]=r[5],t[6]=r[6],t[7]=r[7],t[8]=r[8],this}extractBasis(e,t,r){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),r.setFromMatrix3Column(this,2),this}setFromMatrix4(e){const t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const r=e.elements,o=t.elements,u=this.elements,c=r[0],f=r[3],h=r[6],m=r[1],g=r[4],y=r[7],v=r[2],M=r[5],T=r[8],S=o[0],x=o[3],_=o[6],P=o[1],R=o[4],L=o[7],Y=o[2],U=o[5],N=o[8];return u[0]=c*S+f*P+h*Y,u[3]=c*x+f*R+h*U,u[6]=c*_+f*L+h*N,u[1]=m*S+g*P+y*Y,u[4]=m*x+g*R+y*U,u[7]=m*_+g*L+y*N,u[2]=v*S+M*P+T*Y,u[5]=v*x+M*R+T*U,u[8]=v*_+M*L+T*N,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],f=e[5],h=e[6],m=e[7],g=e[8];return t*c*g-t*f*m-r*u*g+r*f*h+o*u*m-o*c*h}invert(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],f=e[5],h=e[6],m=e[7],g=e[8],y=g*c-f*m,v=f*h-g*u,M=m*u-c*h,T=t*y+r*v+o*M;if(T===0)return this.set(0,0,0,0,0,0,0,0,0);const S=1/T;return e[0]=y*S,e[1]=(o*m-g*r)*S,e[2]=(f*r-o*c)*S,e[3]=v*S,e[4]=(g*t-o*h)*S,e[5]=(o*u-f*t)*S,e[6]=M*S,e[7]=(r*h-m*t)*S,e[8]=(c*t-r*u)*S,this}transpose(){let e;const t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){const t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,r,o,u,c,f){const h=Math.cos(u),m=Math.sin(u);return this.set(r*h,r*m,-r*(h*c+m*f)+c+e,-o*m,o*h,-o*(-m*c+h*f)+f+t,0,0,1),this}scale(e,t){return this.premultiply(rf.makeScale(e,t)),this}rotate(e){return this.premultiply(rf.makeRotation(-e)),this}translate(e,t){return this.premultiply(rf.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,-r,0,r,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){const t=this.elements,r=e.elements;for(let o=0;o<9;o++)if(t[o]!==r[o])return!1;return!0}fromArray(e,t=0){for(let r=0;r<9;r++)this.elements[r]=e[r+t];return this}toArray(e=[],t=0){const r=this.elements;return e[t]=r[0],e[t+1]=r[1],e[t+2]=r[2],e[t+3]=r[3],e[t+4]=r[4],e[t+5]=r[5],e[t+6]=r[6],e[t+7]=r[7],e[t+8]=r[8],e}clone(){return new this.constructor().fromArray(this.elements)}}const rf=new mt;function qg(s){for(let e=s.length-1;e>=0;--e)if(s[e]>=65535)return!0;return!1}function au(s){return document.createElementNS("http://www.w3.org/1999/xhtml",s)}function rx(){const s=au("canvas");return s.style.display="block",s}const wm={};function ta(s){s in wm||(wm[s]=!0,console.warn(s))}function sx(s,e,t){return new Promise(function(r,o){function u(){switch(s.clientWaitSync(e,s.SYNC_FLUSH_COMMANDS_BIT,0)){case s.WAIT_FAILED:o();break;case s.TIMEOUT_EXPIRED:setTimeout(u,t);break;default:r()}}setTimeout(u,t)})}const Tm=new mt().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),Am=new mt().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Xa={[wr]:{transfer:nu,primaries:iu,luminanceCoefficients:[.2126,.7152,.0722],toReference:s=>s,fromReference:s=>s},[ui]:{transfer:Ot,primaries:iu,luminanceCoefficients:[.2126,.7152,.0722],toReference:s=>s.convertSRGBToLinear(),fromReference:s=>s.convertLinearToSRGB()},[fu]:{transfer:nu,primaries:ru,luminanceCoefficients:[.2289,.6917,.0793],toReference:s=>s.applyMatrix3(Am),fromReference:s=>s.applyMatrix3(Tm)},[bd]:{transfer:Ot,primaries:ru,luminanceCoefficients:[.2289,.6917,.0793],toReference:s=>s.convertSRGBToLinear().applyMatrix3(Am),fromReference:s=>s.applyMatrix3(Tm).convertLinearToSRGB()}},ax=new Set([wr,fu]),Rt={enabled:!0,_workingColorSpace:wr,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(s){if(!ax.has(s))throw new Error(`Unsupported working color space, "${s}".`);this._workingColorSpace=s},convert:function(s,e,t){if(this.enabled===!1||e===t||!e||!t)return s;const r=Xa[e].toReference,o=Xa[t].fromReference;return o(r(s))},fromWorkingColorSpace:function(s,e){return this.convert(s,this._workingColorSpace,e)},toWorkingColorSpace:function(s,e){return this.convert(s,e,this._workingColorSpace)},getPrimaries:function(s){return Xa[s].primaries},getTransfer:function(s){return s===vr?nu:Xa[s].transfer},getLuminanceCoefficients:function(s,e=this._workingColorSpace){return s.fromArray(Xa[e].luminanceCoefficients)}};function na(s){return s<.04045?s*.0773993808:Math.pow(s*.9478672986+.0521327014,2.4)}function sf(s){return s<.0031308?s*12.92:1.055*Math.pow(s,.41666)-.055}let Rs;class ox{static getDataURL(e){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let t;if(e instanceof HTMLCanvasElement)t=e;else{Rs===void 0&&(Rs=au("canvas")),Rs.width=e.width,Rs.height=e.height;const r=Rs.getContext("2d");e instanceof ImageData?r.putImageData(e,0,0):r.drawImage(e,0,0,e.width,e.height),t=Rs}return t.width>2048||t.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",e),t.toDataURL("image/jpeg",.6)):t.toDataURL("image/png")}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){const t=au("canvas");t.width=e.width,t.height=e.height;const r=t.getContext("2d");r.drawImage(e,0,0,e.width,e.height);const o=r.getImageData(0,0,e.width,e.height),u=o.data;for(let c=0;c<u.length;c++)u[c]=na(u[c]/255)*255;return r.putImageData(o,0,0),t}else if(e.data){const t=e.data.slice(0);for(let r=0;r<t.length;r++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[r]=Math.floor(na(t[r]/255)*255):t[r]=na(t[r]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}}let lx=0;class $g{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:lx++}),this.uuid=Sr(),this.data=e,this.dataReady=!0,this.version=0}set needsUpdate(e){e===!0&&this.version++}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];const r={uuid:this.uuid,url:""},o=this.data;if(o!==null){let u;if(Array.isArray(o)){u=[];for(let c=0,f=o.length;c<f;c++)o[c].isDataTexture?u.push(af(o[c].image)):u.push(af(o[c]))}else u=af(o);r.url=u}return t||(e.images[this.uuid]=r),r}}function af(s){return typeof HTMLImageElement<"u"&&s instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&s instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&s instanceof ImageBitmap?ox.getDataURL(s):s.data?{data:Array.from(s.data),width:s.width,height:s.height,type:s.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let ux=0;class Mn extends ca{constructor(e=Mn.DEFAULT_IMAGE,t=Mn.DEFAULT_MAPPING,r=Kr,o=Kr,u=ci,c=Zr,f=di,h=Vi,m=Mn.DEFAULT_ANISOTROPY,g=vr){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:ux++}),this.uuid=Sr(),this.name="",this.source=new $g(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=r,this.wrapT=o,this.magFilter=u,this.minFilter=c,this.anisotropy=m,this.format=f,this.internalFormat=null,this.type=h,this.offset=new ht(0,0),this.repeat=new ht(1,1),this.center=new ht(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new mt,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=g,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];const r={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(r.userData=this.userData),t||(e.textures[this.uuid]=r),r}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==Ug)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case Wf:e.x=e.x-Math.floor(e.x);break;case Kr:e.x=e.x<0?0:1;break;case jf:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case Wf:e.y=e.y-Math.floor(e.y);break;case Kr:e.y=e.y<0?0:1;break;case jf:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}}Mn.DEFAULT_IMAGE=null;Mn.DEFAULT_MAPPING=Ug;Mn.DEFAULT_ANISOTROPY=1;class Zt{constructor(e=0,t=0,r=0,o=1){Zt.prototype.isVector4=!0,this.x=e,this.y=t,this.z=r,this.w=o}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,r,o){return this.x=e,this.y=t,this.z=r,this.w=o,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){const t=this.x,r=this.y,o=this.z,u=this.w,c=e.elements;return this.x=c[0]*t+c[4]*r+c[8]*o+c[12]*u,this.y=c[1]*t+c[5]*r+c[9]*o+c[13]*u,this.z=c[2]*t+c[6]*r+c[10]*o+c[14]*u,this.w=c[3]*t+c[7]*r+c[11]*o+c[15]*u,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);const t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,r,o,u;const h=e.elements,m=h[0],g=h[4],y=h[8],v=h[1],M=h[5],T=h[9],S=h[2],x=h[6],_=h[10];if(Math.abs(g-v)<.01&&Math.abs(y-S)<.01&&Math.abs(T-x)<.01){if(Math.abs(g+v)<.1&&Math.abs(y+S)<.1&&Math.abs(T+x)<.1&&Math.abs(m+M+_-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;const R=(m+1)/2,L=(M+1)/2,Y=(_+1)/2,U=(g+v)/4,N=(y+S)/4,H=(T+x)/4;return R>L&&R>Y?R<.01?(r=0,o=.707106781,u=.707106781):(r=Math.sqrt(R),o=U/r,u=N/r):L>Y?L<.01?(r=.707106781,o=0,u=.707106781):(o=Math.sqrt(L),r=U/o,u=H/o):Y<.01?(r=.707106781,o=.707106781,u=0):(u=Math.sqrt(Y),r=N/u,o=H/u),this.set(r,o,u,t),this}let P=Math.sqrt((x-T)*(x-T)+(y-S)*(y-S)+(v-g)*(v-g));return Math.abs(P)<.001&&(P=1),this.x=(x-T)/P,this.y=(y-S)/P,this.z=(v-g)/P,this.w=Math.acos((m+M+_-1)/2),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this.w=t[15],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this.w=Math.max(e.w,Math.min(t.w,this.w)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this.w=Math.max(e,Math.min(t,this.w)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this.z=e.z+(t.z-e.z)*r,this.w=e.w+(t.w-e.w)*r,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class cx extends ca{constructor(e=1,t=1,r={}){super(),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=1,this.scissor=new Zt(0,0,e,t),this.scissorTest=!1,this.viewport=new Zt(0,0,e,t);const o={width:e,height:t,depth:1};r=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:ci,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},r);const u=new Mn(o,r.mapping,r.wrapS,r.wrapT,r.magFilter,r.minFilter,r.format,r.type,r.anisotropy,r.colorSpace);u.flipY=!1,u.generateMipmaps=r.generateMipmaps,u.internalFormat=r.internalFormat,this.textures=[];const c=r.count;for(let f=0;f<c;f++)this.textures[f]=u.clone(),this.textures[f].isRenderTargetTexture=!0;this.depthBuffer=r.depthBuffer,this.stencilBuffer=r.stencilBuffer,this.resolveDepthBuffer=r.resolveDepthBuffer,this.resolveStencilBuffer=r.resolveStencilBuffer,this.depthTexture=r.depthTexture,this.samples=r.samples}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}setSize(e,t,r=1){if(this.width!==e||this.height!==t||this.depth!==r){this.width=e,this.height=t,this.depth=r;for(let o=0,u=this.textures.length;o<u;o++)this.textures[o].image.width=e,this.textures[o].image.height=t,this.textures[o].image.depth=r;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let r=0,o=e.textures.length;r<o;r++)this.textures[r]=e.textures[r].clone(),this.textures[r].isRenderTargetTexture=!0;const t=Object.assign({},e.texture.image);return this.texture.source=new $g(t),this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class ts extends cx{constructor(e=1,t=1,r={}){super(e,t,r),this.isWebGLRenderTarget=!0}}class Kg extends Mn{constructor(e=null,t=1,r=1,o=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:r,depth:o},this.magFilter=Kn,this.minFilter=Kn,this.wrapR=Kr,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}}class fx extends Mn{constructor(e=null,t=1,r=1,o=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:r,depth:o},this.magFilter=Kn,this.minFilter=Kn,this.wrapR=Kr,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class so{constructor(e=0,t=0,r=0,o=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=r,this._w=o}static slerpFlat(e,t,r,o,u,c,f){let h=r[o+0],m=r[o+1],g=r[o+2],y=r[o+3];const v=u[c+0],M=u[c+1],T=u[c+2],S=u[c+3];if(f===0){e[t+0]=h,e[t+1]=m,e[t+2]=g,e[t+3]=y;return}if(f===1){e[t+0]=v,e[t+1]=M,e[t+2]=T,e[t+3]=S;return}if(y!==S||h!==v||m!==M||g!==T){let x=1-f;const _=h*v+m*M+g*T+y*S,P=_>=0?1:-1,R=1-_*_;if(R>Number.EPSILON){const Y=Math.sqrt(R),U=Math.atan2(Y,_*P);x=Math.sin(x*U)/Y,f=Math.sin(f*U)/Y}const L=f*P;if(h=h*x+v*L,m=m*x+M*L,g=g*x+T*L,y=y*x+S*L,x===1-f){const Y=1/Math.sqrt(h*h+m*m+g*g+y*y);h*=Y,m*=Y,g*=Y,y*=Y}}e[t]=h,e[t+1]=m,e[t+2]=g,e[t+3]=y}static multiplyQuaternionsFlat(e,t,r,o,u,c){const f=r[o],h=r[o+1],m=r[o+2],g=r[o+3],y=u[c],v=u[c+1],M=u[c+2],T=u[c+3];return e[t]=f*T+g*y+h*M-m*v,e[t+1]=h*T+g*v+m*y-f*M,e[t+2]=m*T+g*M+f*v-h*y,e[t+3]=g*T-f*y-h*v-m*M,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,r,o){return this._x=e,this._y=t,this._z=r,this._w=o,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){const r=e._x,o=e._y,u=e._z,c=e._order,f=Math.cos,h=Math.sin,m=f(r/2),g=f(o/2),y=f(u/2),v=h(r/2),M=h(o/2),T=h(u/2);switch(c){case"XYZ":this._x=v*g*y+m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y-v*M*T;break;case"YXZ":this._x=v*g*y+m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y+v*M*T;break;case"ZXY":this._x=v*g*y-m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y-v*M*T;break;case"ZYX":this._x=v*g*y-m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y+v*M*T;break;case"YZX":this._x=v*g*y+m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y-v*M*T;break;case"XZY":this._x=v*g*y-m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y+v*M*T;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+c)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){const r=t/2,o=Math.sin(r);return this._x=e.x*o,this._y=e.y*o,this._z=e.z*o,this._w=Math.cos(r),this._onChangeCallback(),this}setFromRotationMatrix(e){const t=e.elements,r=t[0],o=t[4],u=t[8],c=t[1],f=t[5],h=t[9],m=t[2],g=t[6],y=t[10],v=r+f+y;if(v>0){const M=.5/Math.sqrt(v+1);this._w=.25/M,this._x=(g-h)*M,this._y=(u-m)*M,this._z=(c-o)*M}else if(r>f&&r>y){const M=2*Math.sqrt(1+r-f-y);this._w=(g-h)/M,this._x=.25*M,this._y=(o+c)/M,this._z=(u+m)/M}else if(f>y){const M=2*Math.sqrt(1+f-r-y);this._w=(u-m)/M,this._x=(o+c)/M,this._y=.25*M,this._z=(h+g)/M}else{const M=2*Math.sqrt(1+y-r-f);this._w=(c-o)/M,this._x=(u+m)/M,this._y=(h+g)/M,this._z=.25*M}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let r=e.dot(t)+1;return r<Number.EPSILON?(r=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=r):(this._x=0,this._y=-e.z,this._z=e.y,this._w=r)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=r),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(Sn(this.dot(e),-1,1)))}rotateTowards(e,t){const r=this.angleTo(e);if(r===0)return this;const o=Math.min(1,t/r);return this.slerp(e,o),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){const r=e._x,o=e._y,u=e._z,c=e._w,f=t._x,h=t._y,m=t._z,g=t._w;return this._x=r*g+c*f+o*m-u*h,this._y=o*g+c*h+u*f-r*m,this._z=u*g+c*m+r*h-o*f,this._w=c*g-r*f-o*h-u*m,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);const r=this._x,o=this._y,u=this._z,c=this._w;let f=c*e._w+r*e._x+o*e._y+u*e._z;if(f<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,f=-f):this.copy(e),f>=1)return this._w=c,this._x=r,this._y=o,this._z=u,this;const h=1-f*f;if(h<=Number.EPSILON){const M=1-t;return this._w=M*c+t*this._w,this._x=M*r+t*this._x,this._y=M*o+t*this._y,this._z=M*u+t*this._z,this.normalize(),this}const m=Math.sqrt(h),g=Math.atan2(m,f),y=Math.sin((1-t)*g)/m,v=Math.sin(t*g)/m;return this._w=c*y+this._w*v,this._x=r*y+this._x*v,this._y=o*y+this._y*v,this._z=u*y+this._z*v,this._onChangeCallback(),this}slerpQuaternions(e,t,r){return this.copy(e).slerp(t,r)}random(){const e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),r=Math.random(),o=Math.sqrt(1-r),u=Math.sqrt(r);return this.set(o*Math.sin(e),o*Math.cos(e),u*Math.sin(t),u*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class Q{constructor(e=0,t=0,r=0){Q.prototype.isVector3=!0,this.x=e,this.y=t,this.z=r}set(e,t,r){return r===void 0&&(r=this.z),this.x=e,this.y=t,this.z=r,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(Cm.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(Cm.setFromAxisAngle(e,t))}applyMatrix3(e){const t=this.x,r=this.y,o=this.z,u=e.elements;return this.x=u[0]*t+u[3]*r+u[6]*o,this.y=u[1]*t+u[4]*r+u[7]*o,this.z=u[2]*t+u[5]*r+u[8]*o,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){const t=this.x,r=this.y,o=this.z,u=e.elements,c=1/(u[3]*t+u[7]*r+u[11]*o+u[15]);return this.x=(u[0]*t+u[4]*r+u[8]*o+u[12])*c,this.y=(u[1]*t+u[5]*r+u[9]*o+u[13])*c,this.z=(u[2]*t+u[6]*r+u[10]*o+u[14])*c,this}applyQuaternion(e){const t=this.x,r=this.y,o=this.z,u=e.x,c=e.y,f=e.z,h=e.w,m=2*(c*o-f*r),g=2*(f*t-u*o),y=2*(u*r-c*t);return this.x=t+h*m+c*y-f*g,this.y=r+h*g+f*m-u*y,this.z=o+h*y+u*g-c*m,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){const t=this.x,r=this.y,o=this.z,u=e.elements;return this.x=u[0]*t+u[4]*r+u[8]*o,this.y=u[1]*t+u[5]*r+u[9]*o,this.z=u[2]*t+u[6]*r+u[10]*o,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this.z=e.z+(t.z-e.z)*r,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){const r=e.x,o=e.y,u=e.z,c=t.x,f=t.y,h=t.z;return this.x=o*h-u*f,this.y=u*c-r*h,this.z=r*f-o*c,this}projectOnVector(e){const t=e.lengthSq();if(t===0)return this.set(0,0,0);const r=e.dot(this)/t;return this.copy(e).multiplyScalar(r)}projectOnPlane(e){return of.copy(this).projectOnVector(e),this.sub(of)}reflect(e){return this.sub(of.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const r=this.dot(e)/t;return Math.acos(Sn(r,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,r=this.y-e.y,o=this.z-e.z;return t*t+r*r+o*o}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,r){const o=Math.sin(t)*e;return this.x=o*Math.sin(r),this.y=Math.cos(t)*e,this.z=o*Math.cos(r),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,r){return this.x=e*Math.sin(t),this.y=r,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){const t=this.setFromMatrixColumn(e,0).length(),r=this.setFromMatrixColumn(e,1).length(),o=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=r,this.z=o,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const e=Math.random()*Math.PI*2,t=Math.random()*2-1,r=Math.sqrt(1-t*t);return this.x=r*Math.cos(e),this.y=t,this.z=r*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const of=new Q,Cm=new so;class ao{constructor(e=new Q(1/0,1/0,1/0),t=new Q(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,r=e.length;t<r;t+=3)this.expandByPoint(si.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,r=e.count;t<r;t++)this.expandByPoint(si.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,r=e.length;t<r;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){const r=si.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(r),this.max.copy(e).add(r),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);const r=e.geometry;if(r!==void 0){const u=r.getAttribute("position");if(t===!0&&u!==void 0&&e.isInstancedMesh!==!0)for(let c=0,f=u.count;c<f;c++)e.isMesh===!0?e.getVertexPosition(c,si):si.fromBufferAttribute(u,c),si.applyMatrix4(e.matrixWorld),this.expandByPoint(si);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),ml.copy(e.boundingBox)):(r.boundingBox===null&&r.computeBoundingBox(),ml.copy(r.boundingBox)),ml.applyMatrix4(e.matrixWorld),this.union(ml)}const o=e.children;for(let u=0,c=o.length;u<c;u++)this.expandByObject(o[u],t);return this}containsPoint(e){return e.x>=this.min.x&&e.x<=this.max.x&&e.y>=this.min.y&&e.y<=this.max.y&&e.z>=this.min.z&&e.z<=this.max.z}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return e.max.x>=this.min.x&&e.min.x<=this.max.x&&e.max.y>=this.min.y&&e.min.y<=this.max.y&&e.max.z>=this.min.z&&e.min.z<=this.max.z}intersectsSphere(e){return this.clampPoint(e.center,si),si.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,r;return e.normal.x>0?(t=e.normal.x*this.min.x,r=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,r=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,r+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,r+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,r+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,r+=e.normal.z*this.min.z),t<=-e.constant&&r>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(Ya),gl.subVectors(this.max,Ya),bs.subVectors(e.a,Ya),Ps.subVectors(e.b,Ya),Ls.subVectors(e.c,Ya),dr.subVectors(Ps,bs),hr.subVectors(Ls,Ps),Br.subVectors(bs,Ls);let t=[0,-dr.z,dr.y,0,-hr.z,hr.y,0,-Br.z,Br.y,dr.z,0,-dr.x,hr.z,0,-hr.x,Br.z,0,-Br.x,-dr.y,dr.x,0,-hr.y,hr.x,0,-Br.y,Br.x,0];return!lf(t,bs,Ps,Ls,gl)||(t=[1,0,0,0,1,0,0,0,1],!lf(t,bs,Ps,Ls,gl))?!1:(_l.crossVectors(dr,hr),t=[_l.x,_l.y,_l.z],lf(t,bs,Ps,Ls,gl))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,si).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(si).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(Di[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),Di[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),Di[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),Di[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),Di[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),Di[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),Di[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),Di[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(Di),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}}const Di=[new Q,new Q,new Q,new Q,new Q,new Q,new Q,new Q],si=new Q,ml=new ao,bs=new Q,Ps=new Q,Ls=new Q,dr=new Q,hr=new Q,Br=new Q,Ya=new Q,gl=new Q,_l=new Q,zr=new Q;function lf(s,e,t,r,o){for(let u=0,c=s.length-3;u<=c;u+=3){zr.fromArray(s,u);const f=o.x*Math.abs(zr.x)+o.y*Math.abs(zr.y)+o.z*Math.abs(zr.z),h=e.dot(zr),m=t.dot(zr),g=r.dot(zr);if(Math.max(-Math.max(h,m,g),Math.min(h,m,g))>f)return!1}return!0}const dx=new ao,qa=new Q,uf=new Q;class du{constructor(e=new Q,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){const r=this.center;t!==void 0?r.copy(t):dx.setFromPoints(e).getCenter(r);let o=0;for(let u=0,c=e.length;u<c;u++)o=Math.max(o,r.distanceToSquared(e[u]));return this.radius=Math.sqrt(o),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){const t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){const r=this.center.distanceToSquared(e);return t.copy(e),r>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;qa.subVectors(e,this.center);const t=qa.lengthSq();if(t>this.radius*this.radius){const r=Math.sqrt(t),o=(r-this.radius)*.5;this.center.addScaledVector(qa,o/r),this.radius+=o}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(uf.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(qa.copy(e.center).add(uf)),this.expandByPoint(qa.copy(e.center).sub(uf))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}}const Ii=new Q,cf=new Q,vl=new Q,pr=new Q,ff=new Q,xl=new Q,df=new Q;class Zg{constructor(e=new Q,t=new Q(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,Ii)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);const r=t.dot(this.direction);return r<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,r)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){const t=Ii.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(Ii.copy(this.origin).addScaledVector(this.direction,t),Ii.distanceToSquared(e))}distanceSqToSegment(e,t,r,o){cf.copy(e).add(t).multiplyScalar(.5),vl.copy(t).sub(e).normalize(),pr.copy(this.origin).sub(cf);const u=e.distanceTo(t)*.5,c=-this.direction.dot(vl),f=pr.dot(this.direction),h=-pr.dot(vl),m=pr.lengthSq(),g=Math.abs(1-c*c);let y,v,M,T;if(g>0)if(y=c*h-f,v=c*f-h,T=u*g,y>=0)if(v>=-T)if(v<=T){const S=1/g;y*=S,v*=S,M=y*(y+c*v+2*f)+v*(c*y+v+2*h)+m}else v=u,y=Math.max(0,-(c*v+f)),M=-y*y+v*(v+2*h)+m;else v=-u,y=Math.max(0,-(c*v+f)),M=-y*y+v*(v+2*h)+m;else v<=-T?(y=Math.max(0,-(-c*u+f)),v=y>0?-u:Math.min(Math.max(-u,-h),u),M=-y*y+v*(v+2*h)+m):v<=T?(y=0,v=Math.min(Math.max(-u,-h),u),M=v*(v+2*h)+m):(y=Math.max(0,-(c*u+f)),v=y>0?u:Math.min(Math.max(-u,-h),u),M=-y*y+v*(v+2*h)+m);else v=c>0?-u:u,y=Math.max(0,-(c*v+f)),M=-y*y+v*(v+2*h)+m;return r&&r.copy(this.origin).addScaledVector(this.direction,y),o&&o.copy(cf).addScaledVector(vl,v),M}intersectSphere(e,t){Ii.subVectors(e.center,this.origin);const r=Ii.dot(this.direction),o=Ii.dot(Ii)-r*r,u=e.radius*e.radius;if(o>u)return null;const c=Math.sqrt(u-o),f=r-c,h=r+c;return h<0?null:f<0?this.at(h,t):this.at(f,t)}intersectsSphere(e){return this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){const t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;const r=-(this.origin.dot(e.normal)+e.constant)/t;return r>=0?r:null}intersectPlane(e,t){const r=this.distanceToPlane(e);return r===null?null:this.at(r,t)}intersectsPlane(e){const t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let r,o,u,c,f,h;const m=1/this.direction.x,g=1/this.direction.y,y=1/this.direction.z,v=this.origin;return m>=0?(r=(e.min.x-v.x)*m,o=(e.max.x-v.x)*m):(r=(e.max.x-v.x)*m,o=(e.min.x-v.x)*m),g>=0?(u=(e.min.y-v.y)*g,c=(e.max.y-v.y)*g):(u=(e.max.y-v.y)*g,c=(e.min.y-v.y)*g),r>c||u>o||((u>r||isNaN(r))&&(r=u),(c<o||isNaN(o))&&(o=c),y>=0?(f=(e.min.z-v.z)*y,h=(e.max.z-v.z)*y):(f=(e.max.z-v.z)*y,h=(e.min.z-v.z)*y),r>h||f>o)||((f>r||r!==r)&&(r=f),(h<o||o!==o)&&(o=h),o<0)?null:this.at(r>=0?r:o,t)}intersectsBox(e){return this.intersectBox(e,Ii)!==null}intersectTriangle(e,t,r,o,u){ff.subVectors(t,e),xl.subVectors(r,e),df.crossVectors(ff,xl);let c=this.direction.dot(df),f;if(c>0){if(o)return null;f=1}else if(c<0)f=-1,c=-c;else return null;pr.subVectors(this.origin,e);const h=f*this.direction.dot(xl.crossVectors(pr,xl));if(h<0)return null;const m=f*this.direction.dot(ff.cross(pr));if(m<0||h+m>c)return null;const g=-f*pr.dot(df);return g<0?null:this.at(g/c,u)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class Gt{constructor(e,t,r,o,u,c,f,h,m,g,y,v,M,T,S,x){Gt.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,r,o,u,c,f,h,m,g,y,v,M,T,S,x)}set(e,t,r,o,u,c,f,h,m,g,y,v,M,T,S,x){const _=this.elements;return _[0]=e,_[4]=t,_[8]=r,_[12]=o,_[1]=u,_[5]=c,_[9]=f,_[13]=h,_[2]=m,_[6]=g,_[10]=y,_[14]=v,_[3]=M,_[7]=T,_[11]=S,_[15]=x,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new Gt().fromArray(this.elements)}copy(e){const t=this.elements,r=e.elements;return t[0]=r[0],t[1]=r[1],t[2]=r[2],t[3]=r[3],t[4]=r[4],t[5]=r[5],t[6]=r[6],t[7]=r[7],t[8]=r[8],t[9]=r[9],t[10]=r[10],t[11]=r[11],t[12]=r[12],t[13]=r[13],t[14]=r[14],t[15]=r[15],this}copyPosition(e){const t=this.elements,r=e.elements;return t[12]=r[12],t[13]=r[13],t[14]=r[14],this}setFromMatrix3(e){const t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,r){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),r.setFromMatrixColumn(this,2),this}makeBasis(e,t,r){return this.set(e.x,t.x,r.x,0,e.y,t.y,r.y,0,e.z,t.z,r.z,0,0,0,0,1),this}extractRotation(e){const t=this.elements,r=e.elements,o=1/Ns.setFromMatrixColumn(e,0).length(),u=1/Ns.setFromMatrixColumn(e,1).length(),c=1/Ns.setFromMatrixColumn(e,2).length();return t[0]=r[0]*o,t[1]=r[1]*o,t[2]=r[2]*o,t[3]=0,t[4]=r[4]*u,t[5]=r[5]*u,t[6]=r[6]*u,t[7]=0,t[8]=r[8]*c,t[9]=r[9]*c,t[10]=r[10]*c,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){const t=this.elements,r=e.x,o=e.y,u=e.z,c=Math.cos(r),f=Math.sin(r),h=Math.cos(o),m=Math.sin(o),g=Math.cos(u),y=Math.sin(u);if(e.order==="XYZ"){const v=c*g,M=c*y,T=f*g,S=f*y;t[0]=h*g,t[4]=-h*y,t[8]=m,t[1]=M+T*m,t[5]=v-S*m,t[9]=-f*h,t[2]=S-v*m,t[6]=T+M*m,t[10]=c*h}else if(e.order==="YXZ"){const v=h*g,M=h*y,T=m*g,S=m*y;t[0]=v+S*f,t[4]=T*f-M,t[8]=c*m,t[1]=c*y,t[5]=c*g,t[9]=-f,t[2]=M*f-T,t[6]=S+v*f,t[10]=c*h}else if(e.order==="ZXY"){const v=h*g,M=h*y,T=m*g,S=m*y;t[0]=v-S*f,t[4]=-c*y,t[8]=T+M*f,t[1]=M+T*f,t[5]=c*g,t[9]=S-v*f,t[2]=-c*m,t[6]=f,t[10]=c*h}else if(e.order==="ZYX"){const v=c*g,M=c*y,T=f*g,S=f*y;t[0]=h*g,t[4]=T*m-M,t[8]=v*m+S,t[1]=h*y,t[5]=S*m+v,t[9]=M*m-T,t[2]=-m,t[6]=f*h,t[10]=c*h}else if(e.order==="YZX"){const v=c*h,M=c*m,T=f*h,S=f*m;t[0]=h*g,t[4]=S-v*y,t[8]=T*y+M,t[1]=y,t[5]=c*g,t[9]=-f*g,t[2]=-m*g,t[6]=M*y+T,t[10]=v-S*y}else if(e.order==="XZY"){const v=c*h,M=c*m,T=f*h,S=f*m;t[0]=h*g,t[4]=-y,t[8]=m*g,t[1]=v*y+S,t[5]=c*g,t[9]=M*y-T,t[2]=T*y-M,t[6]=f*g,t[10]=S*y+v}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(hx,e,px)}lookAt(e,t,r){const o=this.elements;return kn.subVectors(e,t),kn.lengthSq()===0&&(kn.z=1),kn.normalize(),mr.crossVectors(r,kn),mr.lengthSq()===0&&(Math.abs(r.z)===1?kn.x+=1e-4:kn.z+=1e-4,kn.normalize(),mr.crossVectors(r,kn)),mr.normalize(),yl.crossVectors(kn,mr),o[0]=mr.x,o[4]=yl.x,o[8]=kn.x,o[1]=mr.y,o[5]=yl.y,o[9]=kn.y,o[2]=mr.z,o[6]=yl.z,o[10]=kn.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const r=e.elements,o=t.elements,u=this.elements,c=r[0],f=r[4],h=r[8],m=r[12],g=r[1],y=r[5],v=r[9],M=r[13],T=r[2],S=r[6],x=r[10],_=r[14],P=r[3],R=r[7],L=r[11],Y=r[15],U=o[0],N=o[4],H=o[8],b=o[12],w=o[1],I=o[5],K=o[9],V=o[13],Z=o[2],te=o[6],ue=o[10],se=o[14],G=o[3],he=o[7],fe=o[11],k=o[15];return u[0]=c*U+f*w+h*Z+m*G,u[4]=c*N+f*I+h*te+m*he,u[8]=c*H+f*K+h*ue+m*fe,u[12]=c*b+f*V+h*se+m*k,u[1]=g*U+y*w+v*Z+M*G,u[5]=g*N+y*I+v*te+M*he,u[9]=g*H+y*K+v*ue+M*fe,u[13]=g*b+y*V+v*se+M*k,u[2]=T*U+S*w+x*Z+_*G,u[6]=T*N+S*I+x*te+_*he,u[10]=T*H+S*K+x*ue+_*fe,u[14]=T*b+S*V+x*se+_*k,u[3]=P*U+R*w+L*Z+Y*G,u[7]=P*N+R*I+L*te+Y*he,u[11]=P*H+R*K+L*ue+Y*fe,u[15]=P*b+R*V+L*se+Y*k,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){const e=this.elements,t=e[0],r=e[4],o=e[8],u=e[12],c=e[1],f=e[5],h=e[9],m=e[13],g=e[2],y=e[6],v=e[10],M=e[14],T=e[3],S=e[7],x=e[11],_=e[15];return T*(+u*h*y-o*m*y-u*f*v+r*m*v+o*f*M-r*h*M)+S*(+t*h*M-t*m*v+u*c*v-o*c*M+o*m*g-u*h*g)+x*(+t*m*y-t*f*M-u*c*y+r*c*M+u*f*g-r*m*g)+_*(-o*f*g-t*h*y+t*f*v+o*c*y-r*c*v+r*h*g)}transpose(){const e=this.elements;let t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,r){const o=this.elements;return e.isVector3?(o[12]=e.x,o[13]=e.y,o[14]=e.z):(o[12]=e,o[13]=t,o[14]=r),this}invert(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],f=e[5],h=e[6],m=e[7],g=e[8],y=e[9],v=e[10],M=e[11],T=e[12],S=e[13],x=e[14],_=e[15],P=y*x*m-S*v*m+S*h*M-f*x*M-y*h*_+f*v*_,R=T*v*m-g*x*m-T*h*M+c*x*M+g*h*_-c*v*_,L=g*S*m-T*y*m+T*f*M-c*S*M-g*f*_+c*y*_,Y=T*y*h-g*S*h-T*f*v+c*S*v+g*f*x-c*y*x,U=t*P+r*R+o*L+u*Y;if(U===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const N=1/U;return e[0]=P*N,e[1]=(S*v*u-y*x*u-S*o*M+r*x*M+y*o*_-r*v*_)*N,e[2]=(f*x*u-S*h*u+S*o*m-r*x*m-f*o*_+r*h*_)*N,e[3]=(y*h*u-f*v*u-y*o*m+r*v*m+f*o*M-r*h*M)*N,e[4]=R*N,e[5]=(g*x*u-T*v*u+T*o*M-t*x*M-g*o*_+t*v*_)*N,e[6]=(T*h*u-c*x*u-T*o*m+t*x*m+c*o*_-t*h*_)*N,e[7]=(c*v*u-g*h*u+g*o*m-t*v*m-c*o*M+t*h*M)*N,e[8]=L*N,e[9]=(T*y*u-g*S*u-T*r*M+t*S*M+g*r*_-t*y*_)*N,e[10]=(c*S*u-T*f*u+T*r*m-t*S*m-c*r*_+t*f*_)*N,e[11]=(g*f*u-c*y*u-g*r*m+t*y*m+c*r*M-t*f*M)*N,e[12]=Y*N,e[13]=(g*S*o-T*y*o+T*r*v-t*S*v-g*r*x+t*y*x)*N,e[14]=(T*f*o-c*S*o-T*r*h+t*S*h+c*r*x-t*f*x)*N,e[15]=(c*y*o-g*f*o+g*r*h-t*y*h-c*r*v+t*f*v)*N,this}scale(e){const t=this.elements,r=e.x,o=e.y,u=e.z;return t[0]*=r,t[4]*=o,t[8]*=u,t[1]*=r,t[5]*=o,t[9]*=u,t[2]*=r,t[6]*=o,t[10]*=u,t[3]*=r,t[7]*=o,t[11]*=u,this}getMaxScaleOnAxis(){const e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],r=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],o=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,r,o))}makeTranslation(e,t,r){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,r,0,0,0,1),this}makeRotationX(e){const t=Math.cos(e),r=Math.sin(e);return this.set(1,0,0,0,0,t,-r,0,0,r,t,0,0,0,0,1),this}makeRotationY(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,0,r,0,0,1,0,0,-r,0,t,0,0,0,0,1),this}makeRotationZ(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,-r,0,0,r,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){const r=Math.cos(t),o=Math.sin(t),u=1-r,c=e.x,f=e.y,h=e.z,m=u*c,g=u*f;return this.set(m*c+r,m*f-o*h,m*h+o*f,0,m*f+o*h,g*f+r,g*h-o*c,0,m*h-o*f,g*h+o*c,u*h*h+r,0,0,0,0,1),this}makeScale(e,t,r){return this.set(e,0,0,0,0,t,0,0,0,0,r,0,0,0,0,1),this}makeShear(e,t,r,o,u,c){return this.set(1,r,u,0,e,1,c,0,t,o,1,0,0,0,0,1),this}compose(e,t,r){const o=this.elements,u=t._x,c=t._y,f=t._z,h=t._w,m=u+u,g=c+c,y=f+f,v=u*m,M=u*g,T=u*y,S=c*g,x=c*y,_=f*y,P=h*m,R=h*g,L=h*y,Y=r.x,U=r.y,N=r.z;return o[0]=(1-(S+_))*Y,o[1]=(M+L)*Y,o[2]=(T-R)*Y,o[3]=0,o[4]=(M-L)*U,o[5]=(1-(v+_))*U,o[6]=(x+P)*U,o[7]=0,o[8]=(T+R)*N,o[9]=(x-P)*N,o[10]=(1-(v+S))*N,o[11]=0,o[12]=e.x,o[13]=e.y,o[14]=e.z,o[15]=1,this}decompose(e,t,r){const o=this.elements;let u=Ns.set(o[0],o[1],o[2]).length();const c=Ns.set(o[4],o[5],o[6]).length(),f=Ns.set(o[8],o[9],o[10]).length();this.determinant()<0&&(u=-u),e.x=o[12],e.y=o[13],e.z=o[14],ai.copy(this);const m=1/u,g=1/c,y=1/f;return ai.elements[0]*=m,ai.elements[1]*=m,ai.elements[2]*=m,ai.elements[4]*=g,ai.elements[5]*=g,ai.elements[6]*=g,ai.elements[8]*=y,ai.elements[9]*=y,ai.elements[10]*=y,t.setFromRotationMatrix(ai),r.x=u,r.y=c,r.z=f,this}makePerspective(e,t,r,o,u,c,f=Hi){const h=this.elements,m=2*u/(t-e),g=2*u/(r-o),y=(t+e)/(t-e),v=(r+o)/(r-o);let M,T;if(f===Hi)M=-(c+u)/(c-u),T=-2*c*u/(c-u);else if(f===su)M=-c/(c-u),T=-c*u/(c-u);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+f);return h[0]=m,h[4]=0,h[8]=y,h[12]=0,h[1]=0,h[5]=g,h[9]=v,h[13]=0,h[2]=0,h[6]=0,h[10]=M,h[14]=T,h[3]=0,h[7]=0,h[11]=-1,h[15]=0,this}makeOrthographic(e,t,r,o,u,c,f=Hi){const h=this.elements,m=1/(t-e),g=1/(r-o),y=1/(c-u),v=(t+e)*m,M=(r+o)*g;let T,S;if(f===Hi)T=(c+u)*y,S=-2*y;else if(f===su)T=u*y,S=-1*y;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+f);return h[0]=2*m,h[4]=0,h[8]=0,h[12]=-v,h[1]=0,h[5]=2*g,h[9]=0,h[13]=-M,h[2]=0,h[6]=0,h[10]=S,h[14]=-T,h[3]=0,h[7]=0,h[11]=0,h[15]=1,this}equals(e){const t=this.elements,r=e.elements;for(let o=0;o<16;o++)if(t[o]!==r[o])return!1;return!0}fromArray(e,t=0){for(let r=0;r<16;r++)this.elements[r]=e[r+t];return this}toArray(e=[],t=0){const r=this.elements;return e[t]=r[0],e[t+1]=r[1],e[t+2]=r[2],e[t+3]=r[3],e[t+4]=r[4],e[t+5]=r[5],e[t+6]=r[6],e[t+7]=r[7],e[t+8]=r[8],e[t+9]=r[9],e[t+10]=r[10],e[t+11]=r[11],e[t+12]=r[12],e[t+13]=r[13],e[t+14]=r[14],e[t+15]=r[15],e}}const Ns=new Q,ai=new Gt,hx=new Q(0,0,0),px=new Q(1,1,1),mr=new Q,yl=new Q,kn=new Q,Rm=new Gt,bm=new so;class Si{constructor(e=0,t=0,r=0,o=Si.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=r,this._order=o}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,r,o=this._order){return this._x=e,this._y=t,this._z=r,this._order=o,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,r=!0){const o=e.elements,u=o[0],c=o[4],f=o[8],h=o[1],m=o[5],g=o[9],y=o[2],v=o[6],M=o[10];switch(t){case"XYZ":this._y=Math.asin(Sn(f,-1,1)),Math.abs(f)<.9999999?(this._x=Math.atan2(-g,M),this._z=Math.atan2(-c,u)):(this._x=Math.atan2(v,m),this._z=0);break;case"YXZ":this._x=Math.asin(-Sn(g,-1,1)),Math.abs(g)<.9999999?(this._y=Math.atan2(f,M),this._z=Math.atan2(h,m)):(this._y=Math.atan2(-y,u),this._z=0);break;case"ZXY":this._x=Math.asin(Sn(v,-1,1)),Math.abs(v)<.9999999?(this._y=Math.atan2(-y,M),this._z=Math.atan2(-c,m)):(this._y=0,this._z=Math.atan2(h,u));break;case"ZYX":this._y=Math.asin(-Sn(y,-1,1)),Math.abs(y)<.9999999?(this._x=Math.atan2(v,M),this._z=Math.atan2(h,u)):(this._x=0,this._z=Math.atan2(-c,m));break;case"YZX":this._z=Math.asin(Sn(h,-1,1)),Math.abs(h)<.9999999?(this._x=Math.atan2(-g,m),this._y=Math.atan2(-y,u)):(this._x=0,this._y=Math.atan2(f,M));break;case"XZY":this._z=Math.asin(-Sn(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(v,m),this._y=Math.atan2(f,u)):(this._x=Math.atan2(-g,M),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,r===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,r){return Rm.makeRotationFromQuaternion(e),this.setFromRotationMatrix(Rm,t,r)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return bm.setFromEuler(this),this.setFromQuaternion(bm,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}Si.DEFAULT_ORDER="XYZ";class Qg{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}}let mx=0;const Pm=new Q,Ds=new so,Ui=new Gt,Sl=new Q,$a=new Q,gx=new Q,_x=new so,Lm=new Q(1,0,0),Nm=new Q(0,1,0),Dm=new Q(0,0,1),Im={type:"added"},vx={type:"removed"},Is={type:"childadded",child:null},hf={type:"childremoved",child:null};class Qt extends ca{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:mx++}),this.uuid=Sr(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Qt.DEFAULT_UP.clone();const e=new Q,t=new Si,r=new so,o=new Q(1,1,1);function u(){r.setFromEuler(t,!1)}function c(){t.setFromQuaternion(r,void 0,!1)}t._onChange(u),r._onChange(c),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:r},scale:{configurable:!0,enumerable:!0,value:o},modelViewMatrix:{value:new Gt},normalMatrix:{value:new mt}}),this.matrix=new Gt,this.matrixWorld=new Gt,this.matrixAutoUpdate=Qt.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Qt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Qg,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return Ds.setFromAxisAngle(e,t),this.quaternion.multiply(Ds),this}rotateOnWorldAxis(e,t){return Ds.setFromAxisAngle(e,t),this.quaternion.premultiply(Ds),this}rotateX(e){return this.rotateOnAxis(Lm,e)}rotateY(e){return this.rotateOnAxis(Nm,e)}rotateZ(e){return this.rotateOnAxis(Dm,e)}translateOnAxis(e,t){return Pm.copy(e).applyQuaternion(this.quaternion),this.position.add(Pm.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(Lm,e)}translateY(e){return this.translateOnAxis(Nm,e)}translateZ(e){return this.translateOnAxis(Dm,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(Ui.copy(this.matrixWorld).invert())}lookAt(e,t,r){e.isVector3?Sl.copy(e):Sl.set(e,t,r);const o=this.parent;this.updateWorldMatrix(!0,!1),$a.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Ui.lookAt($a,Sl,this.up):Ui.lookAt(Sl,$a,this.up),this.quaternion.setFromRotationMatrix(Ui),o&&(Ui.extractRotation(o.matrixWorld),Ds.setFromRotationMatrix(Ui),this.quaternion.premultiply(Ds.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(Im),Is.child=e,this.dispatchEvent(Is),Is.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let r=0;r<arguments.length;r++)this.remove(arguments[r]);return this}const t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(vx),hf.child=e,this.dispatchEvent(hf),hf.child=null),this}removeFromParent(){const e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),Ui.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),Ui.multiply(e.parent.matrixWorld)),e.applyMatrix4(Ui),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(Im),Is.child=e,this.dispatchEvent(Is),Is.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let r=0,o=this.children.length;r<o;r++){const c=this.children[r].getObjectByProperty(e,t);if(c!==void 0)return c}}getObjectsByProperty(e,t,r=[]){this[e]===t&&r.push(this);const o=this.children;for(let u=0,c=o.length;u<c;u++)o[u].getObjectsByProperty(e,t,r);return r}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose($a,e,gx),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose($a,_x,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);const t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].traverseVisible(e)}traverseAncestors(e){const t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,e=!0);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].updateMatrixWorld(e)}updateWorldMatrix(e,t){const r=this.parent;if(e===!0&&r!==null&&r.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),t===!0){const o=this.children;for(let u=0,c=o.length;u<c;u++)o[u].updateWorldMatrix(!1,!0)}}toJSON(e){const t=e===void 0||typeof e=="string",r={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},r.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const o={};o.uuid=this.uuid,o.type=this.type,this.name!==""&&(o.name=this.name),this.castShadow===!0&&(o.castShadow=!0),this.receiveShadow===!0&&(o.receiveShadow=!0),this.visible===!1&&(o.visible=!1),this.frustumCulled===!1&&(o.frustumCulled=!1),this.renderOrder!==0&&(o.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(o.userData=this.userData),o.layers=this.layers.mask,o.matrix=this.matrix.toArray(),o.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(o.matrixAutoUpdate=!1),this.isInstancedMesh&&(o.type="InstancedMesh",o.count=this.count,o.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(o.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(o.type="BatchedMesh",o.perObjectFrustumCulled=this.perObjectFrustumCulled,o.sortObjects=this.sortObjects,o.drawRanges=this._drawRanges,o.reservedRanges=this._reservedRanges,o.visibility=this._visibility,o.active=this._active,o.bounds=this._bounds.map(f=>({boxInitialized:f.boxInitialized,boxMin:f.box.min.toArray(),boxMax:f.box.max.toArray(),sphereInitialized:f.sphereInitialized,sphereRadius:f.sphere.radius,sphereCenter:f.sphere.center.toArray()})),o.maxInstanceCount=this._maxInstanceCount,o.maxVertexCount=this._maxVertexCount,o.maxIndexCount=this._maxIndexCount,o.geometryInitialized=this._geometryInitialized,o.geometryCount=this._geometryCount,o.matricesTexture=this._matricesTexture.toJSON(e),this._colorsTexture!==null&&(o.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(o.boundingSphere={center:o.boundingSphere.center.toArray(),radius:o.boundingSphere.radius}),this.boundingBox!==null&&(o.boundingBox={min:o.boundingBox.min.toArray(),max:o.boundingBox.max.toArray()}));function u(f,h){return f[h.uuid]===void 0&&(f[h.uuid]=h.toJSON(e)),h.uuid}if(this.isScene)this.background&&(this.background.isColor?o.background=this.background.toJSON():this.background.isTexture&&(o.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(o.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){o.geometry=u(e.geometries,this.geometry);const f=this.geometry.parameters;if(f!==void 0&&f.shapes!==void 0){const h=f.shapes;if(Array.isArray(h))for(let m=0,g=h.length;m<g;m++){const y=h[m];u(e.shapes,y)}else u(e.shapes,h)}}if(this.isSkinnedMesh&&(o.bindMode=this.bindMode,o.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(u(e.skeletons,this.skeleton),o.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const f=[];for(let h=0,m=this.material.length;h<m;h++)f.push(u(e.materials,this.material[h]));o.material=f}else o.material=u(e.materials,this.material);if(this.children.length>0){o.children=[];for(let f=0;f<this.children.length;f++)o.children.push(this.children[f].toJSON(e).object)}if(this.animations.length>0){o.animations=[];for(let f=0;f<this.animations.length;f++){const h=this.animations[f];o.animations.push(u(e.animations,h))}}if(t){const f=c(e.geometries),h=c(e.materials),m=c(e.textures),g=c(e.images),y=c(e.shapes),v=c(e.skeletons),M=c(e.animations),T=c(e.nodes);f.length>0&&(r.geometries=f),h.length>0&&(r.materials=h),m.length>0&&(r.textures=m),g.length>0&&(r.images=g),y.length>0&&(r.shapes=y),v.length>0&&(r.skeletons=v),M.length>0&&(r.animations=M),T.length>0&&(r.nodes=T)}return r.object=o,r;function c(f){const h=[];for(const m in f){const g=f[m];delete g.metadata,h.push(g)}return h}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let r=0;r<e.children.length;r++){const o=e.children[r];this.add(o.clone())}return this}}Qt.DEFAULT_UP=new Q(0,1,0);Qt.DEFAULT_MATRIX_AUTO_UPDATE=!0;Qt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const oi=new Q,Fi=new Q,pf=new Q,Oi=new Q,Us=new Q,Fs=new Q,Um=new Q,mf=new Q,gf=new Q,_f=new Q;class fi{constructor(e=new Q,t=new Q,r=new Q){this.a=e,this.b=t,this.c=r}static getNormal(e,t,r,o){o.subVectors(r,t),oi.subVectors(e,t),o.cross(oi);const u=o.lengthSq();return u>0?o.multiplyScalar(1/Math.sqrt(u)):o.set(0,0,0)}static getBarycoord(e,t,r,o,u){oi.subVectors(o,t),Fi.subVectors(r,t),pf.subVectors(e,t);const c=oi.dot(oi),f=oi.dot(Fi),h=oi.dot(pf),m=Fi.dot(Fi),g=Fi.dot(pf),y=c*m-f*f;if(y===0)return u.set(0,0,0),null;const v=1/y,M=(m*h-f*g)*v,T=(c*g-f*h)*v;return u.set(1-M-T,T,M)}static containsPoint(e,t,r,o){return this.getBarycoord(e,t,r,o,Oi)===null?!1:Oi.x>=0&&Oi.y>=0&&Oi.x+Oi.y<=1}static getInterpolation(e,t,r,o,u,c,f,h){return this.getBarycoord(e,t,r,o,Oi)===null?(h.x=0,h.y=0,"z"in h&&(h.z=0),"w"in h&&(h.w=0),null):(h.setScalar(0),h.addScaledVector(u,Oi.x),h.addScaledVector(c,Oi.y),h.addScaledVector(f,Oi.z),h)}static isFrontFacing(e,t,r,o){return oi.subVectors(r,t),Fi.subVectors(e,t),oi.cross(Fi).dot(o)<0}set(e,t,r){return this.a.copy(e),this.b.copy(t),this.c.copy(r),this}setFromPointsAndIndices(e,t,r,o){return this.a.copy(e[t]),this.b.copy(e[r]),this.c.copy(e[o]),this}setFromAttributeAndIndices(e,t,r,o){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,r),this.c.fromBufferAttribute(e,o),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return oi.subVectors(this.c,this.b),Fi.subVectors(this.a,this.b),oi.cross(Fi).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return fi.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return fi.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,r,o,u){return fi.getInterpolation(e,this.a,this.b,this.c,t,r,o,u)}containsPoint(e){return fi.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return fi.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){const r=this.a,o=this.b,u=this.c;let c,f;Us.subVectors(o,r),Fs.subVectors(u,r),mf.subVectors(e,r);const h=Us.dot(mf),m=Fs.dot(mf);if(h<=0&&m<=0)return t.copy(r);gf.subVectors(e,o);const g=Us.dot(gf),y=Fs.dot(gf);if(g>=0&&y<=g)return t.copy(o);const v=h*y-g*m;if(v<=0&&h>=0&&g<=0)return c=h/(h-g),t.copy(r).addScaledVector(Us,c);_f.subVectors(e,u);const M=Us.dot(_f),T=Fs.dot(_f);if(T>=0&&M<=T)return t.copy(u);const S=M*m-h*T;if(S<=0&&m>=0&&T<=0)return f=m/(m-T),t.copy(r).addScaledVector(Fs,f);const x=g*T-M*y;if(x<=0&&y-g>=0&&M-T>=0)return Um.subVectors(u,o),f=(y-g)/(y-g+(M-T)),t.copy(o).addScaledVector(Um,f);const _=1/(x+S+v);return c=S*_,f=v*_,t.copy(r).addScaledVector(Us,c).addScaledVector(Fs,f)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}}const Jg={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},gr={h:0,s:0,l:0},Ml={h:0,s:0,l:0};function vf(s,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?s+(e-s)*6*t:t<1/2?e:t<2/3?s+(e-s)*6*(2/3-t):s}class yt{constructor(e,t,r){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,r)}set(e,t,r){if(t===void 0&&r===void 0){const o=e;o&&o.isColor?this.copy(o):typeof o=="number"?this.setHex(o):typeof o=="string"&&this.setStyle(o)}else this.setRGB(e,t,r);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=ui){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,Rt.toWorkingColorSpace(this,t),this}setRGB(e,t,r,o=Rt.workingColorSpace){return this.r=e,this.g=t,this.b=r,Rt.toWorkingColorSpace(this,o),this}setHSL(e,t,r,o=Rt.workingColorSpace){if(e=ix(e,1),t=Sn(t,0,1),r=Sn(r,0,1),t===0)this.r=this.g=this.b=r;else{const u=r<=.5?r*(1+t):r+t-r*t,c=2*r-u;this.r=vf(c,u,e+1/3),this.g=vf(c,u,e),this.b=vf(c,u,e-1/3)}return Rt.toWorkingColorSpace(this,o),this}setStyle(e,t=ui){function r(u){u!==void 0&&parseFloat(u)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let o;if(o=/^(\w+)\(([^\)]*)\)/.exec(e)){let u;const c=o[1],f=o[2];switch(c){case"rgb":case"rgba":if(u=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(f))return r(u[4]),this.setRGB(Math.min(255,parseInt(u[1],10))/255,Math.min(255,parseInt(u[2],10))/255,Math.min(255,parseInt(u[3],10))/255,t);if(u=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(f))return r(u[4]),this.setRGB(Math.min(100,parseInt(u[1],10))/100,Math.min(100,parseInt(u[2],10))/100,Math.min(100,parseInt(u[3],10))/100,t);break;case"hsl":case"hsla":if(u=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(f))return r(u[4]),this.setHSL(parseFloat(u[1])/360,parseFloat(u[2])/100,parseFloat(u[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(o=/^\#([A-Fa-f\d]+)$/.exec(e)){const u=o[1],c=u.length;if(c===3)return this.setRGB(parseInt(u.charAt(0),16)/15,parseInt(u.charAt(1),16)/15,parseInt(u.charAt(2),16)/15,t);if(c===6)return this.setHex(parseInt(u,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=ui){const r=Jg[e.toLowerCase()];return r!==void 0?this.setHex(r,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=na(e.r),this.g=na(e.g),this.b=na(e.b),this}copyLinearToSRGB(e){return this.r=sf(e.r),this.g=sf(e.g),this.b=sf(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=ui){return Rt.fromWorkingColorSpace(mn.copy(this),e),Math.round(Sn(mn.r*255,0,255))*65536+Math.round(Sn(mn.g*255,0,255))*256+Math.round(Sn(mn.b*255,0,255))}getHexString(e=ui){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=Rt.workingColorSpace){Rt.fromWorkingColorSpace(mn.copy(this),t);const r=mn.r,o=mn.g,u=mn.b,c=Math.max(r,o,u),f=Math.min(r,o,u);let h,m;const g=(f+c)/2;if(f===c)h=0,m=0;else{const y=c-f;switch(m=g<=.5?y/(c+f):y/(2-c-f),c){case r:h=(o-u)/y+(o<u?6:0);break;case o:h=(u-r)/y+2;break;case u:h=(r-o)/y+4;break}h/=6}return e.h=h,e.s=m,e.l=g,e}getRGB(e,t=Rt.workingColorSpace){return Rt.fromWorkingColorSpace(mn.copy(this),t),e.r=mn.r,e.g=mn.g,e.b=mn.b,e}getStyle(e=ui){Rt.fromWorkingColorSpace(mn.copy(this),e);const t=mn.r,r=mn.g,o=mn.b;return e!==ui?`color(${e} ${t.toFixed(3)} ${r.toFixed(3)} ${o.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(r*255)},${Math.round(o*255)})`}offsetHSL(e,t,r){return this.getHSL(gr),this.setHSL(gr.h+e,gr.s+t,gr.l+r)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,r){return this.r=e.r+(t.r-e.r)*r,this.g=e.g+(t.g-e.g)*r,this.b=e.b+(t.b-e.b)*r,this}lerpHSL(e,t){this.getHSL(gr),e.getHSL(Ml);const r=nf(gr.h,Ml.h,t),o=nf(gr.s,Ml.s,t),u=nf(gr.l,Ml.l,t);return this.setHSL(r,o,u),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){const t=this.r,r=this.g,o=this.b,u=e.elements;return this.r=u[0]*t+u[3]*r+u[6]*o,this.g=u[1]*t+u[4]*r+u[7]*o,this.b=u[2]*t+u[5]*r+u[8]*o,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const mn=new yt;yt.NAMES=Jg;let xx=0;class is extends ca{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:xx++}),this.uuid=Sr(),this.name="",this.type="Material",this.blending=Js,this.side=Mr,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=zf,this.blendDst=Hf,this.blendEquation=qr,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new yt(0,0,0),this.blendAlpha=0,this.depthFunc=tu,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=Mm,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=Cs,this.stencilZFail=Cs,this.stencilZPass=Cs,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(const t in e){const r=e[t];if(r===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}const o=this[t];if(o===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}o&&o.isColor?o.set(r):o&&o.isVector3&&r&&r.isVector3?o.copy(r):this[t]=r}}toJSON(e){const t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});const r={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};r.uuid=this.uuid,r.type=this.type,this.name!==""&&(r.name=this.name),this.color&&this.color.isColor&&(r.color=this.color.getHex()),this.roughness!==void 0&&(r.roughness=this.roughness),this.metalness!==void 0&&(r.metalness=this.metalness),this.sheen!==void 0&&(r.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(r.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(r.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(r.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(r.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(r.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(r.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(r.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(r.shininess=this.shininess),this.clearcoat!==void 0&&(r.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(r.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(r.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(r.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(r.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,r.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(r.dispersion=this.dispersion),this.iridescence!==void 0&&(r.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(r.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(r.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(r.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(r.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(r.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(r.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(r.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(r.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(r.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(r.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(r.lightMap=this.lightMap.toJSON(e).uuid,r.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(r.aoMap=this.aoMap.toJSON(e).uuid,r.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(r.bumpMap=this.bumpMap.toJSON(e).uuid,r.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(r.normalMap=this.normalMap.toJSON(e).uuid,r.normalMapType=this.normalMapType,r.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(r.displacementMap=this.displacementMap.toJSON(e).uuid,r.displacementScale=this.displacementScale,r.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(r.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(r.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(r.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(r.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(r.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(r.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(r.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(r.combine=this.combine)),this.envMapRotation!==void 0&&(r.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(r.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(r.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(r.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(r.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(r.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(r.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(r.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(r.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(r.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(r.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(r.size=this.size),this.shadowSide!==null&&(r.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(r.sizeAttenuation=this.sizeAttenuation),this.blending!==Js&&(r.blending=this.blending),this.side!==Mr&&(r.side=this.side),this.vertexColors===!0&&(r.vertexColors=!0),this.opacity<1&&(r.opacity=this.opacity),this.transparent===!0&&(r.transparent=!0),this.blendSrc!==zf&&(r.blendSrc=this.blendSrc),this.blendDst!==Hf&&(r.blendDst=this.blendDst),this.blendEquation!==qr&&(r.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(r.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(r.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(r.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(r.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(r.blendAlpha=this.blendAlpha),this.depthFunc!==tu&&(r.depthFunc=this.depthFunc),this.depthTest===!1&&(r.depthTest=this.depthTest),this.depthWrite===!1&&(r.depthWrite=this.depthWrite),this.colorWrite===!1&&(r.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(r.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==Mm&&(r.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(r.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(r.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==Cs&&(r.stencilFail=this.stencilFail),this.stencilZFail!==Cs&&(r.stencilZFail=this.stencilZFail),this.stencilZPass!==Cs&&(r.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(r.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(r.rotation=this.rotation),this.polygonOffset===!0&&(r.polygonOffset=!0),this.polygonOffsetFactor!==0&&(r.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(r.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(r.linewidth=this.linewidth),this.dashSize!==void 0&&(r.dashSize=this.dashSize),this.gapSize!==void 0&&(r.gapSize=this.gapSize),this.scale!==void 0&&(r.scale=this.scale),this.dithering===!0&&(r.dithering=!0),this.alphaTest>0&&(r.alphaTest=this.alphaTest),this.alphaHash===!0&&(r.alphaHash=!0),this.alphaToCoverage===!0&&(r.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(r.premultipliedAlpha=!0),this.forceSinglePass===!0&&(r.forceSinglePass=!0),this.wireframe===!0&&(r.wireframe=!0),this.wireframeLinewidth>1&&(r.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(r.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(r.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(r.flatShading=!0),this.visible===!1&&(r.visible=!1),this.toneMapped===!1&&(r.toneMapped=!1),this.fog===!1&&(r.fog=!1),Object.keys(this.userData).length>0&&(r.userData=this.userData);function o(u){const c=[];for(const f in u){const h=u[f];delete h.metadata,c.push(h)}return c}if(t){const u=o(e.textures),c=o(e.images);u.length>0&&(r.textures=u),c.length>0&&(r.images=c)}return r}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;const t=e.clippingPlanes;let r=null;if(t!==null){const o=t.length;r=new Array(o);for(let u=0;u!==o;++u)r[u]=t[u].clone()}return this.clippingPlanes=r,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}onBuild(){console.warn("Material: onBuild() has been removed.")}onBeforeRender(){console.warn("Material: onBeforeRender() has been removed.")}}class Pd extends is{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new yt(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Si,this.combine=Ig,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}}const Yt=new Q,El=new ht;class hi{constructor(e,t,r=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=r,this.usage=vd,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=zi,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return ta("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,r){e*=this.itemSize,r*=t.itemSize;for(let o=0,u=this.itemSize;o<u;o++)this.array[e+o]=t.array[r+o];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,r=this.count;t<r;t++)El.fromBufferAttribute(this,t),El.applyMatrix3(e),this.setXY(t,El.x,El.y);else if(this.itemSize===3)for(let t=0,r=this.count;t<r;t++)Yt.fromBufferAttribute(this,t),Yt.applyMatrix3(e),this.setXYZ(t,Yt.x,Yt.y,Yt.z);return this}applyMatrix4(e){for(let t=0,r=this.count;t<r;t++)Yt.fromBufferAttribute(this,t),Yt.applyMatrix4(e),this.setXYZ(t,Yt.x,Yt.y,Yt.z);return this}applyNormalMatrix(e){for(let t=0,r=this.count;t<r;t++)Yt.fromBufferAttribute(this,t),Yt.applyNormalMatrix(e),this.setXYZ(t,Yt.x,Yt.y,Yt.z);return this}transformDirection(e){for(let t=0,r=this.count;t<r;t++)Yt.fromBufferAttribute(this,t),Yt.transformDirection(e),this.setXYZ(t,Yt.x,Yt.y,Yt.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let r=this.array[e*this.itemSize+t];return this.normalized&&(r=xi(r,this.array)),r}setComponent(e,t,r){return this.normalized&&(r=Lt(r,this.array)),this.array[e*this.itemSize+t]=r,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=xi(t,this.array)),t}setX(e,t){return this.normalized&&(t=Lt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=xi(t,this.array)),t}setY(e,t){return this.normalized&&(t=Lt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=xi(t,this.array)),t}setZ(e,t){return this.normalized&&(t=Lt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=xi(t,this.array)),t}setW(e,t){return this.normalized&&(t=Lt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,r){return e*=this.itemSize,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array)),this.array[e+0]=t,this.array[e+1]=r,this}setXYZ(e,t,r,o){return e*=this.itemSize,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array),o=Lt(o,this.array)),this.array[e+0]=t,this.array[e+1]=r,this.array[e+2]=o,this}setXYZW(e,t,r,o,u){return e*=this.itemSize,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array),o=Lt(o,this.array),u=Lt(u,this.array)),this.array[e+0]=t,this.array[e+1]=r,this.array[e+2]=o,this.array[e+3]=u,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==vd&&(e.usage=this.usage),e}}class e_ extends hi{constructor(e,t,r){super(new Uint16Array(e),t,r)}}class t_ extends hi{constructor(e,t,r){super(new Uint32Array(e),t,r)}}class gn extends hi{constructor(e,t,r){super(new Float32Array(e),t,r)}}let yx=0;const qn=new Gt,xf=new Qt,Os=new Q,Bn=new ao,Ka=new ao,on=new Q;class Hn extends ca{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:yx++}),this.uuid=Sr(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(qg(e)?t_:e_)(e,1):this.index=e,this}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,r=0){this.groups.push({start:e,count:t,materialIndex:r})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){const t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);const r=this.attributes.normal;if(r!==void 0){const u=new mt().getNormalMatrix(e);r.applyNormalMatrix(u),r.needsUpdate=!0}const o=this.attributes.tangent;return o!==void 0&&(o.transformDirection(e),o.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return qn.makeRotationFromQuaternion(e),this.applyMatrix4(qn),this}rotateX(e){return qn.makeRotationX(e),this.applyMatrix4(qn),this}rotateY(e){return qn.makeRotationY(e),this.applyMatrix4(qn),this}rotateZ(e){return qn.makeRotationZ(e),this.applyMatrix4(qn),this}translate(e,t,r){return qn.makeTranslation(e,t,r),this.applyMatrix4(qn),this}scale(e,t,r){return qn.makeScale(e,t,r),this.applyMatrix4(qn),this}lookAt(e){return xf.lookAt(e),xf.updateMatrix(),this.applyMatrix4(xf.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Os).negate(),this.translate(Os.x,Os.y,Os.z),this}setFromPoints(e){const t=[];for(let r=0,o=e.length;r<o;r++){const u=e[r];t.push(u.x,u.y,u.z||0)}return this.setAttribute("position",new gn(t,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new ao);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new Q(-1/0,-1/0,-1/0),new Q(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let r=0,o=t.length;r<o;r++){const u=t[r];Bn.setFromBufferAttribute(u),this.morphTargetsRelative?(on.addVectors(this.boundingBox.min,Bn.min),this.boundingBox.expandByPoint(on),on.addVectors(this.boundingBox.max,Bn.max),this.boundingBox.expandByPoint(on)):(this.boundingBox.expandByPoint(Bn.min),this.boundingBox.expandByPoint(Bn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new du);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new Q,1/0);return}if(e){const r=this.boundingSphere.center;if(Bn.setFromBufferAttribute(e),t)for(let u=0,c=t.length;u<c;u++){const f=t[u];Ka.setFromBufferAttribute(f),this.morphTargetsRelative?(on.addVectors(Bn.min,Ka.min),Bn.expandByPoint(on),on.addVectors(Bn.max,Ka.max),Bn.expandByPoint(on)):(Bn.expandByPoint(Ka.min),Bn.expandByPoint(Ka.max))}Bn.getCenter(r);let o=0;for(let u=0,c=e.count;u<c;u++)on.fromBufferAttribute(e,u),o=Math.max(o,r.distanceToSquared(on));if(t)for(let u=0,c=t.length;u<c;u++){const f=t[u],h=this.morphTargetsRelative;for(let m=0,g=f.count;m<g;m++)on.fromBufferAttribute(f,m),h&&(Os.fromBufferAttribute(e,m),on.add(Os)),o=Math.max(o,r.distanceToSquared(on))}this.boundingSphere.radius=Math.sqrt(o),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const r=t.position,o=t.normal,u=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new hi(new Float32Array(4*r.count),4));const c=this.getAttribute("tangent"),f=[],h=[];for(let H=0;H<r.count;H++)f[H]=new Q,h[H]=new Q;const m=new Q,g=new Q,y=new Q,v=new ht,M=new ht,T=new ht,S=new Q,x=new Q;function _(H,b,w){m.fromBufferAttribute(r,H),g.fromBufferAttribute(r,b),y.fromBufferAttribute(r,w),v.fromBufferAttribute(u,H),M.fromBufferAttribute(u,b),T.fromBufferAttribute(u,w),g.sub(m),y.sub(m),M.sub(v),T.sub(v);const I=1/(M.x*T.y-T.x*M.y);isFinite(I)&&(S.copy(g).multiplyScalar(T.y).addScaledVector(y,-M.y).multiplyScalar(I),x.copy(y).multiplyScalar(M.x).addScaledVector(g,-T.x).multiplyScalar(I),f[H].add(S),f[b].add(S),f[w].add(S),h[H].add(x),h[b].add(x),h[w].add(x))}let P=this.groups;P.length===0&&(P=[{start:0,count:e.count}]);for(let H=0,b=P.length;H<b;++H){const w=P[H],I=w.start,K=w.count;for(let V=I,Z=I+K;V<Z;V+=3)_(e.getX(V+0),e.getX(V+1),e.getX(V+2))}const R=new Q,L=new Q,Y=new Q,U=new Q;function N(H){Y.fromBufferAttribute(o,H),U.copy(Y);const b=f[H];R.copy(b),R.sub(Y.multiplyScalar(Y.dot(b))).normalize(),L.crossVectors(U,b);const I=L.dot(h[H])<0?-1:1;c.setXYZW(H,R.x,R.y,R.z,I)}for(let H=0,b=P.length;H<b;++H){const w=P[H],I=w.start,K=w.count;for(let V=I,Z=I+K;V<Z;V+=3)N(e.getX(V+0)),N(e.getX(V+1)),N(e.getX(V+2))}}computeVertexNormals(){const e=this.index,t=this.getAttribute("position");if(t!==void 0){let r=this.getAttribute("normal");if(r===void 0)r=new hi(new Float32Array(t.count*3),3),this.setAttribute("normal",r);else for(let v=0,M=r.count;v<M;v++)r.setXYZ(v,0,0,0);const o=new Q,u=new Q,c=new Q,f=new Q,h=new Q,m=new Q,g=new Q,y=new Q;if(e)for(let v=0,M=e.count;v<M;v+=3){const T=e.getX(v+0),S=e.getX(v+1),x=e.getX(v+2);o.fromBufferAttribute(t,T),u.fromBufferAttribute(t,S),c.fromBufferAttribute(t,x),g.subVectors(c,u),y.subVectors(o,u),g.cross(y),f.fromBufferAttribute(r,T),h.fromBufferAttribute(r,S),m.fromBufferAttribute(r,x),f.add(g),h.add(g),m.add(g),r.setXYZ(T,f.x,f.y,f.z),r.setXYZ(S,h.x,h.y,h.z),r.setXYZ(x,m.x,m.y,m.z)}else for(let v=0,M=t.count;v<M;v+=3)o.fromBufferAttribute(t,v+0),u.fromBufferAttribute(t,v+1),c.fromBufferAttribute(t,v+2),g.subVectors(c,u),y.subVectors(o,u),g.cross(y),r.setXYZ(v+0,g.x,g.y,g.z),r.setXYZ(v+1,g.x,g.y,g.z),r.setXYZ(v+2,g.x,g.y,g.z);this.normalizeNormals(),r.needsUpdate=!0}}normalizeNormals(){const e=this.attributes.normal;for(let t=0,r=e.count;t<r;t++)on.fromBufferAttribute(e,t),on.normalize(),e.setXYZ(t,on.x,on.y,on.z)}toNonIndexed(){function e(f,h){const m=f.array,g=f.itemSize,y=f.normalized,v=new m.constructor(h.length*g);let M=0,T=0;for(let S=0,x=h.length;S<x;S++){f.isInterleavedBufferAttribute?M=h[S]*f.data.stride+f.offset:M=h[S]*g;for(let _=0;_<g;_++)v[T++]=m[M++]}return new hi(v,g,y)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const t=new Hn,r=this.index.array,o=this.attributes;for(const f in o){const h=o[f],m=e(h,r);t.setAttribute(f,m)}const u=this.morphAttributes;for(const f in u){const h=[],m=u[f];for(let g=0,y=m.length;g<y;g++){const v=m[g],M=e(v,r);h.push(M)}t.morphAttributes[f]=h}t.morphTargetsRelative=this.morphTargetsRelative;const c=this.groups;for(let f=0,h=c.length;f<h;f++){const m=c[f];t.addGroup(m.start,m.count,m.materialIndex)}return t}toJSON(){const e={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){const h=this.parameters;for(const m in h)h[m]!==void 0&&(e[m]=h[m]);return e}e.data={attributes:{}};const t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});const r=this.attributes;for(const h in r){const m=r[h];e.data.attributes[h]=m.toJSON(e.data)}const o={};let u=!1;for(const h in this.morphAttributes){const m=this.morphAttributes[h],g=[];for(let y=0,v=m.length;y<v;y++){const M=m[y];g.push(M.toJSON(e.data))}g.length>0&&(o[h]=g,u=!0)}u&&(e.data.morphAttributes=o,e.data.morphTargetsRelative=this.morphTargetsRelative);const c=this.groups;c.length>0&&(e.data.groups=JSON.parse(JSON.stringify(c)));const f=this.boundingSphere;return f!==null&&(e.data.boundingSphere={center:f.center.toArray(),radius:f.radius}),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const t={};this.name=e.name;const r=e.index;r!==null&&this.setIndex(r.clone(t));const o=e.attributes;for(const m in o){const g=o[m];this.setAttribute(m,g.clone(t))}const u=e.morphAttributes;for(const m in u){const g=[],y=u[m];for(let v=0,M=y.length;v<M;v++)g.push(y[v].clone(t));this.morphAttributes[m]=g}this.morphTargetsRelative=e.morphTargetsRelative;const c=e.groups;for(let m=0,g=c.length;m<g;m++){const y=c[m];this.addGroup(y.start,y.count,y.materialIndex)}const f=e.boundingBox;f!==null&&(this.boundingBox=f.clone());const h=e.boundingSphere;return h!==null&&(this.boundingSphere=h.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const Fm=new Gt,Hr=new Zg,wl=new du,Om=new Q,ks=new Q,Bs=new Q,zs=new Q,yf=new Q,Tl=new Q,Al=new ht,Cl=new ht,Rl=new ht,km=new Q,Bm=new Q,zm=new Q,bl=new Q,Pl=new Q;class zn extends Qt{constructor(e=new Hn,t=new Pd){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){const t=this.geometry.morphAttributes,r=Object.keys(t);if(r.length>0){const o=t[r[0]];if(o!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let u=0,c=o.length;u<c;u++){const f=o[u].name||String(u);this.morphTargetInfluences.push(0),this.morphTargetDictionary[f]=u}}}}getVertexPosition(e,t){const r=this.geometry,o=r.attributes.position,u=r.morphAttributes.position,c=r.morphTargetsRelative;t.fromBufferAttribute(o,e);const f=this.morphTargetInfluences;if(u&&f){Tl.set(0,0,0);for(let h=0,m=u.length;h<m;h++){const g=f[h],y=u[h];g!==0&&(yf.fromBufferAttribute(y,e),c?Tl.addScaledVector(yf,g):Tl.addScaledVector(yf.sub(t),g))}t.add(Tl)}return t}raycast(e,t){const r=this.geometry,o=this.material,u=this.matrixWorld;o!==void 0&&(r.boundingSphere===null&&r.computeBoundingSphere(),wl.copy(r.boundingSphere),wl.applyMatrix4(u),Hr.copy(e.ray).recast(e.near),!(wl.containsPoint(Hr.origin)===!1&&(Hr.intersectSphere(wl,Om)===null||Hr.origin.distanceToSquared(Om)>(e.far-e.near)**2))&&(Fm.copy(u).invert(),Hr.copy(e.ray).applyMatrix4(Fm),!(r.boundingBox!==null&&Hr.intersectsBox(r.boundingBox)===!1)&&this._computeIntersections(e,t,Hr)))}_computeIntersections(e,t,r){let o;const u=this.geometry,c=this.material,f=u.index,h=u.attributes.position,m=u.attributes.uv,g=u.attributes.uv1,y=u.attributes.normal,v=u.groups,M=u.drawRange;if(f!==null)if(Array.isArray(c))for(let T=0,S=v.length;T<S;T++){const x=v[T],_=c[x.materialIndex],P=Math.max(x.start,M.start),R=Math.min(f.count,Math.min(x.start+x.count,M.start+M.count));for(let L=P,Y=R;L<Y;L+=3){const U=f.getX(L),N=f.getX(L+1),H=f.getX(L+2);o=Ll(this,_,e,r,m,g,y,U,N,H),o&&(o.faceIndex=Math.floor(L/3),o.face.materialIndex=x.materialIndex,t.push(o))}}else{const T=Math.max(0,M.start),S=Math.min(f.count,M.start+M.count);for(let x=T,_=S;x<_;x+=3){const P=f.getX(x),R=f.getX(x+1),L=f.getX(x+2);o=Ll(this,c,e,r,m,g,y,P,R,L),o&&(o.faceIndex=Math.floor(x/3),t.push(o))}}else if(h!==void 0)if(Array.isArray(c))for(let T=0,S=v.length;T<S;T++){const x=v[T],_=c[x.materialIndex],P=Math.max(x.start,M.start),R=Math.min(h.count,Math.min(x.start+x.count,M.start+M.count));for(let L=P,Y=R;L<Y;L+=3){const U=L,N=L+1,H=L+2;o=Ll(this,_,e,r,m,g,y,U,N,H),o&&(o.faceIndex=Math.floor(L/3),o.face.materialIndex=x.materialIndex,t.push(o))}}else{const T=Math.max(0,M.start),S=Math.min(h.count,M.start+M.count);for(let x=T,_=S;x<_;x+=3){const P=x,R=x+1,L=x+2;o=Ll(this,c,e,r,m,g,y,P,R,L),o&&(o.faceIndex=Math.floor(x/3),t.push(o))}}}}function Sx(s,e,t,r,o,u,c,f){let h;if(e.side===Ln?h=r.intersectTriangle(c,u,o,!0,f):h=r.intersectTriangle(o,u,c,e.side===Mr,f),h===null)return null;Pl.copy(f),Pl.applyMatrix4(s.matrixWorld);const m=t.ray.origin.distanceTo(Pl);return m<t.near||m>t.far?null:{distance:m,point:Pl.clone(),object:s}}function Ll(s,e,t,r,o,u,c,f,h,m){s.getVertexPosition(f,ks),s.getVertexPosition(h,Bs),s.getVertexPosition(m,zs);const g=Sx(s,e,t,r,ks,Bs,zs,bl);if(g){o&&(Al.fromBufferAttribute(o,f),Cl.fromBufferAttribute(o,h),Rl.fromBufferAttribute(o,m),g.uv=fi.getInterpolation(bl,ks,Bs,zs,Al,Cl,Rl,new ht)),u&&(Al.fromBufferAttribute(u,f),Cl.fromBufferAttribute(u,h),Rl.fromBufferAttribute(u,m),g.uv1=fi.getInterpolation(bl,ks,Bs,zs,Al,Cl,Rl,new ht)),c&&(km.fromBufferAttribute(c,f),Bm.fromBufferAttribute(c,h),zm.fromBufferAttribute(c,m),g.normal=fi.getInterpolation(bl,ks,Bs,zs,km,Bm,zm,new Q),g.normal.dot(r.direction)>0&&g.normal.multiplyScalar(-1));const y={a:f,b:h,c:m,normal:new Q,materialIndex:0};fi.getNormal(ks,Bs,zs,y.normal),g.face=y}return g}class ns extends Hn{constructor(e=1,t=1,r=1,o=1,u=1,c=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:r,widthSegments:o,heightSegments:u,depthSegments:c};const f=this;o=Math.floor(o),u=Math.floor(u),c=Math.floor(c);const h=[],m=[],g=[],y=[];let v=0,M=0;T("z","y","x",-1,-1,r,t,e,c,u,0),T("z","y","x",1,-1,r,t,-e,c,u,1),T("x","z","y",1,1,e,r,t,o,c,2),T("x","z","y",1,-1,e,r,-t,o,c,3),T("x","y","z",1,-1,e,t,r,o,u,4),T("x","y","z",-1,-1,e,t,-r,o,u,5),this.setIndex(h),this.setAttribute("position",new gn(m,3)),this.setAttribute("normal",new gn(g,3)),this.setAttribute("uv",new gn(y,2));function T(S,x,_,P,R,L,Y,U,N,H,b){const w=L/N,I=Y/H,K=L/2,V=Y/2,Z=U/2,te=N+1,ue=H+1;let se=0,G=0;const he=new Q;for(let fe=0;fe<ue;fe++){const k=fe*I-V;for(let ce=0;ce<te;ce++){const ke=ce*w-K;he[S]=ke*P,he[x]=k*R,he[_]=Z,m.push(he.x,he.y,he.z),he[S]=0,he[x]=0,he[_]=U>0?1:-1,g.push(he.x,he.y,he.z),y.push(ce/N),y.push(1-fe/H),se+=1}}for(let fe=0;fe<H;fe++)for(let k=0;k<N;k++){const ce=v+k+te*fe,ke=v+k+te*(fe+1),J=v+(k+1)+te*(fe+1),de=v+(k+1)+te*fe;h.push(ce,ke,de),h.push(ke,J,de),G+=6}f.addGroup(M,G,b),M+=G,v+=se}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new ns(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}}function ua(s){const e={};for(const t in s){e[t]={};for(const r in s[t]){const o=s[t][r];o&&(o.isColor||o.isMatrix3||o.isMatrix4||o.isVector2||o.isVector3||o.isVector4||o.isTexture||o.isQuaternion)?o.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][r]=null):e[t][r]=o.clone():Array.isArray(o)?e[t][r]=o.slice():e[t][r]=o}}return e}function yn(s){const e={};for(let t=0;t<s.length;t++){const r=ua(s[t]);for(const o in r)e[o]=r[o]}return e}function Mx(s){const e=[];for(let t=0;t<s.length;t++)e.push(s[t].clone());return e}function n_(s){const e=s.getRenderTarget();return e===null?s.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:Rt.workingColorSpace}const Ex={clone:ua,merge:yn};var wx=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Tx=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Er extends is{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=wx,this.fragmentShader=Tx,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=ua(e.uniforms),this.uniformsGroups=Mx(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){const t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(const o in this.uniforms){const c=this.uniforms[o].value;c&&c.isTexture?t.uniforms[o]={type:"t",value:c.toJSON(e).uuid}:c&&c.isColor?t.uniforms[o]={type:"c",value:c.getHex()}:c&&c.isVector2?t.uniforms[o]={type:"v2",value:c.toArray()}:c&&c.isVector3?t.uniforms[o]={type:"v3",value:c.toArray()}:c&&c.isVector4?t.uniforms[o]={type:"v4",value:c.toArray()}:c&&c.isMatrix3?t.uniforms[o]={type:"m3",value:c.toArray()}:c&&c.isMatrix4?t.uniforms[o]={type:"m4",value:c.toArray()}:t.uniforms[o]={value:c}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;const r={};for(const o in this.extensions)this.extensions[o]===!0&&(r[o]=!0);return Object.keys(r).length>0&&(t.extensions=r),t}}class i_ extends Qt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new Gt,this.projectionMatrix=new Gt,this.projectionMatrixInverse=new Gt,this.coordinateSystem=Hi}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const _r=new Q,Hm=new ht,Vm=new ht;class $n extends i_{constructor(e=50,t=1,r=.1,o=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=r,this.far=o,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){const t=.5*this.getFilmHeight()/e;this.fov=xd*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){const e=Math.tan(tf*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return xd*2*Math.atan(Math.tan(tf*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,r){_r.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(_r.x,_r.y).multiplyScalar(-e/_r.z),_r.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),r.set(_r.x,_r.y).multiplyScalar(-e/_r.z)}getViewSize(e,t){return this.getViewBounds(e,Hm,Vm),t.subVectors(Vm,Hm)}setViewOffset(e,t,r,o,u,c){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=r,this.view.offsetY=o,this.view.width=u,this.view.height=c,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=this.near;let t=e*Math.tan(tf*.5*this.fov)/this.zoom,r=2*t,o=this.aspect*r,u=-.5*o;const c=this.view;if(this.view!==null&&this.view.enabled){const h=c.fullWidth,m=c.fullHeight;u+=c.offsetX*o/h,t-=c.offsetY*r/m,o*=c.width/h,r*=c.height/m}const f=this.filmOffset;f!==0&&(u+=e*f/this.getFilmWidth()),this.projectionMatrix.makePerspective(u,u+o,t,t-r,e,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}}const Hs=-90,Vs=1;class Ax extends Qt{constructor(e,t,r){super(),this.type="CubeCamera",this.renderTarget=r,this.coordinateSystem=null,this.activeMipmapLevel=0;const o=new $n(Hs,Vs,e,t);o.layers=this.layers,this.add(o);const u=new $n(Hs,Vs,e,t);u.layers=this.layers,this.add(u);const c=new $n(Hs,Vs,e,t);c.layers=this.layers,this.add(c);const f=new $n(Hs,Vs,e,t);f.layers=this.layers,this.add(f);const h=new $n(Hs,Vs,e,t);h.layers=this.layers,this.add(h);const m=new $n(Hs,Vs,e,t);m.layers=this.layers,this.add(m)}updateCoordinateSystem(){const e=this.coordinateSystem,t=this.children.concat(),[r,o,u,c,f,h]=t;for(const m of t)this.remove(m);if(e===Hi)r.up.set(0,1,0),r.lookAt(1,0,0),o.up.set(0,1,0),o.lookAt(-1,0,0),u.up.set(0,0,-1),u.lookAt(0,1,0),c.up.set(0,0,1),c.lookAt(0,-1,0),f.up.set(0,1,0),f.lookAt(0,0,1),h.up.set(0,1,0),h.lookAt(0,0,-1);else if(e===su)r.up.set(0,-1,0),r.lookAt(-1,0,0),o.up.set(0,-1,0),o.lookAt(1,0,0),u.up.set(0,0,1),u.lookAt(0,1,0),c.up.set(0,0,-1),c.lookAt(0,-1,0),f.up.set(0,-1,0),f.lookAt(0,0,1),h.up.set(0,-1,0),h.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(const m of t)this.add(m),m.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();const{renderTarget:r,activeMipmapLevel:o}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());const[u,c,f,h,m,g]=this.children,y=e.getRenderTarget(),v=e.getActiveCubeFace(),M=e.getActiveMipmapLevel(),T=e.xr.enabled;e.xr.enabled=!1;const S=r.texture.generateMipmaps;r.texture.generateMipmaps=!1,e.setRenderTarget(r,0,o),e.render(t,u),e.setRenderTarget(r,1,o),e.render(t,c),e.setRenderTarget(r,2,o),e.render(t,f),e.setRenderTarget(r,3,o),e.render(t,h),e.setRenderTarget(r,4,o),e.render(t,m),r.texture.generateMipmaps=S,e.setRenderTarget(r,5,o),e.render(t,g),e.setRenderTarget(y,v,M),e.xr.enabled=T,r.texture.needsPMREMUpdate=!0}}class r_ extends Mn{constructor(e,t,r,o,u,c,f,h,m,g){e=e!==void 0?e:[],t=t!==void 0?t:sa,super(e,t,r,o,u,c,f,h,m,g),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}}class Cx extends ts{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;const r={width:e,height:e,depth:1},o=[r,r,r,r,r,r];this.texture=new r_(o,t.mapping,t.wrapS,t.wrapT,t.magFilter,t.minFilter,t.format,t.type,t.anisotropy,t.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=t.generateMipmaps!==void 0?t.generateMipmaps:!1,this.texture.minFilter=t.minFilter!==void 0?t.minFilter:ci}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;const r={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},o=new ns(5,5,5),u=new Er({name:"CubemapFromEquirect",uniforms:ua(r.uniforms),vertexShader:r.vertexShader,fragmentShader:r.fragmentShader,side:Ln,blending:xr});u.uniforms.tEquirect.value=t;const c=new zn(o,u),f=t.minFilter;return t.minFilter===Zr&&(t.minFilter=ci),new Ax(1,10,this).update(e,c),t.minFilter=f,c.geometry.dispose(),c.material.dispose(),this}clear(e,t,r,o){const u=e.getRenderTarget();for(let c=0;c<6;c++)e.setRenderTarget(this,c),e.clear(t,r,o);e.setRenderTarget(u)}}const Sf=new Q,Rx=new Q,bx=new mt;class jr{constructor(e=new Q(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,r,o){return this.normal.set(e,t,r),this.constant=o,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,r){const o=Sf.subVectors(r,t).cross(Rx.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(o,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){const e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){const r=e.delta(Sf),o=this.normal.dot(r);if(o===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;const u=-(e.start.dot(this.normal)+this.constant)/o;return u<0||u>1?null:t.copy(e.start).addScaledVector(r,u)}intersectsLine(e){const t=this.distanceToPoint(e.start),r=this.distanceToPoint(e.end);return t<0&&r>0||r<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){const r=t||bx.getNormalMatrix(e),o=this.coplanarPoint(Sf).applyMatrix4(e),u=this.normal.applyMatrix3(r).normalize();return this.constant=-o.dot(u),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Vr=new du,Nl=new Q;class Ld{constructor(e=new jr,t=new jr,r=new jr,o=new jr,u=new jr,c=new jr){this.planes=[e,t,r,o,u,c]}set(e,t,r,o,u,c){const f=this.planes;return f[0].copy(e),f[1].copy(t),f[2].copy(r),f[3].copy(o),f[4].copy(u),f[5].copy(c),this}copy(e){const t=this.planes;for(let r=0;r<6;r++)t[r].copy(e.planes[r]);return this}setFromProjectionMatrix(e,t=Hi){const r=this.planes,o=e.elements,u=o[0],c=o[1],f=o[2],h=o[3],m=o[4],g=o[5],y=o[6],v=o[7],M=o[8],T=o[9],S=o[10],x=o[11],_=o[12],P=o[13],R=o[14],L=o[15];if(r[0].setComponents(h-u,v-m,x-M,L-_).normalize(),r[1].setComponents(h+u,v+m,x+M,L+_).normalize(),r[2].setComponents(h+c,v+g,x+T,L+P).normalize(),r[3].setComponents(h-c,v-g,x-T,L-P).normalize(),r[4].setComponents(h-f,v-y,x-S,L-R).normalize(),t===Hi)r[5].setComponents(h+f,v+y,x+S,L+R).normalize();else if(t===su)r[5].setComponents(f,y,S,R).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),Vr.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{const t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),Vr.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(Vr)}intersectsSprite(e){return Vr.center.set(0,0,0),Vr.radius=.7071067811865476,Vr.applyMatrix4(e.matrixWorld),this.intersectsSphere(Vr)}intersectsSphere(e){const t=this.planes,r=e.center,o=-e.radius;for(let u=0;u<6;u++)if(t[u].distanceToPoint(r)<o)return!1;return!0}intersectsBox(e){const t=this.planes;for(let r=0;r<6;r++){const o=t[r];if(Nl.x=o.normal.x>0?e.max.x:e.min.x,Nl.y=o.normal.y>0?e.max.y:e.min.y,Nl.z=o.normal.z>0?e.max.z:e.min.z,o.distanceToPoint(Nl)<0)return!1}return!0}containsPoint(e){const t=this.planes;for(let r=0;r<6;r++)if(t[r].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function s_(){let s=null,e=!1,t=null,r=null;function o(u,c){t(u,c),r=s.requestAnimationFrame(o)}return{start:function(){e!==!0&&t!==null&&(r=s.requestAnimationFrame(o),e=!0)},stop:function(){s.cancelAnimationFrame(r),e=!1},setAnimationLoop:function(u){t=u},setContext:function(u){s=u}}}function Px(s){const e=new WeakMap;function t(f,h){const m=f.array,g=f.usage,y=m.byteLength,v=s.createBuffer();s.bindBuffer(h,v),s.bufferData(h,m,g),f.onUploadCallback();let M;if(m instanceof Float32Array)M=s.FLOAT;else if(m instanceof Uint16Array)f.isFloat16BufferAttribute?M=s.HALF_FLOAT:M=s.UNSIGNED_SHORT;else if(m instanceof Int16Array)M=s.SHORT;else if(m instanceof Uint32Array)M=s.UNSIGNED_INT;else if(m instanceof Int32Array)M=s.INT;else if(m instanceof Int8Array)M=s.BYTE;else if(m instanceof Uint8Array)M=s.UNSIGNED_BYTE;else if(m instanceof Uint8ClampedArray)M=s.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+m);return{buffer:v,type:M,bytesPerElement:m.BYTES_PER_ELEMENT,version:f.version,size:y}}function r(f,h,m){const g=h.array,y=h._updateRange,v=h.updateRanges;if(s.bindBuffer(m,f),y.count===-1&&v.length===0&&s.bufferSubData(m,0,g),v.length!==0){for(let M=0,T=v.length;M<T;M++){const S=v[M];s.bufferSubData(m,S.start*g.BYTES_PER_ELEMENT,g,S.start,S.count)}h.clearUpdateRanges()}y.count!==-1&&(s.bufferSubData(m,y.offset*g.BYTES_PER_ELEMENT,g,y.offset,y.count),y.count=-1),h.onUploadCallback()}function o(f){return f.isInterleavedBufferAttribute&&(f=f.data),e.get(f)}function u(f){f.isInterleavedBufferAttribute&&(f=f.data);const h=e.get(f);h&&(s.deleteBuffer(h.buffer),e.delete(f))}function c(f,h){if(f.isInterleavedBufferAttribute&&(f=f.data),f.isGLBufferAttribute){const g=e.get(f);(!g||g.version<f.version)&&e.set(f,{buffer:f.buffer,type:f.type,bytesPerElement:f.elementSize,version:f.version});return}const m=e.get(f);if(m===void 0)e.set(f,t(f,h));else if(m.version<f.version){if(m.size!==f.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");r(m.buffer,f,h),m.version=f.version}}return{get:o,remove:u,update:c}}class hu extends Hn{constructor(e=1,t=1,r=1,o=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:r,heightSegments:o};const u=e/2,c=t/2,f=Math.floor(r),h=Math.floor(o),m=f+1,g=h+1,y=e/f,v=t/h,M=[],T=[],S=[],x=[];for(let _=0;_<g;_++){const P=_*v-c;for(let R=0;R<m;R++){const L=R*y-u;T.push(L,-P,0),S.push(0,0,1),x.push(R/f),x.push(1-_/h)}}for(let _=0;_<h;_++)for(let P=0;P<f;P++){const R=P+m*_,L=P+m*(_+1),Y=P+1+m*(_+1),U=P+1+m*_;M.push(R,L,U),M.push(L,Y,U)}this.setIndex(M),this.setAttribute("position",new gn(T,3)),this.setAttribute("normal",new gn(S,3)),this.setAttribute("uv",new gn(x,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new hu(e.width,e.height,e.widthSegments,e.heightSegments)}}var Lx=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Nx=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,Dx=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,Ix=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Ux=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,Fx=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Ox=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,kx=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Bx=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,zx=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,Hx=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Vx=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Gx=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,Wx=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,jx=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,Xx=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,Yx=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,qx=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,$x=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,Kx=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Zx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Qx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,Jx=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,ey=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,ty=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,ny=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,iy=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,ry=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,sy=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,ay=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,oy="gl_FragColor = linearToOutputTexel( gl_FragColor );",ly=`
const mat3 LINEAR_SRGB_TO_LINEAR_DISPLAY_P3 = mat3(
	vec3( 0.8224621, 0.177538, 0.0 ),
	vec3( 0.0331941, 0.9668058, 0.0 ),
	vec3( 0.0170827, 0.0723974, 0.9105199 )
);
const mat3 LINEAR_DISPLAY_P3_TO_LINEAR_SRGB = mat3(
	vec3( 1.2249401, - 0.2249404, 0.0 ),
	vec3( - 0.0420569, 1.0420571, 0.0 ),
	vec3( - 0.0196376, - 0.0786361, 1.0982735 )
);
vec4 LinearSRGBToLinearDisplayP3( in vec4 value ) {
	return vec4( value.rgb * LINEAR_SRGB_TO_LINEAR_DISPLAY_P3, value.a );
}
vec4 LinearDisplayP3ToLinearSRGB( in vec4 value ) {
	return vec4( value.rgb * LINEAR_DISPLAY_P3_TO_LINEAR_SRGB, value.a );
}
vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}`,uy=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,cy=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,fy=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,dy=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,hy=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,py=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,my=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,gy=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,_y=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,vy=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,xy=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,yy=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,Sy=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,My=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,Ey=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,wy=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,Ty=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,Ay=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,Cy=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,Ry=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,by=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,Py=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,Ly=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,Ny=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Dy=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,Iy=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Uy=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Fy=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Oy=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,ky=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,By=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,zy=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Hy=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,Vy=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Gy=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,Wy=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,jy=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Xy=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,Yy=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,qy=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,$y=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,Ky=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Zy=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Qy=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,Jy=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,eS=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,tS=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,nS=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,iS=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,rS=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,sS=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;const float ShiftRight8 = 1. / 256.;
const float Inv255 = 1. / 255.;
const vec4 PackFactors = vec4( 1.0, 256.0, 256.0 * 256.0, 256.0 * 256.0 * 256.0 );
const vec2 UnpackFactors2 = vec2( UnpackDownscale, 1.0 / PackFactors.g );
const vec3 UnpackFactors3 = vec3( UnpackDownscale / PackFactors.rg, 1.0 / PackFactors.b );
const vec4 UnpackFactors4 = vec4( UnpackDownscale / PackFactors.rgb, 1.0 / PackFactors.a );
vec4 packDepthToRGBA( const in float v ) {
	if( v <= 0.0 )
		return vec4( 0., 0., 0., 0. );
	if( v >= 1.0 )
		return vec4( 1., 1., 1., 1. );
	float vuf;
	float af = modf( v * PackFactors.a, vuf );
	float bf = modf( vuf * ShiftRight8, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec4( vuf * Inv255, gf * PackUpscale, bf * PackUpscale, af );
}
vec3 packDepthToRGB( const in float v ) {
	if( v <= 0.0 )
		return vec3( 0., 0., 0. );
	if( v >= 1.0 )
		return vec3( 1., 1., 1. );
	float vuf;
	float bf = modf( v * PackFactors.b, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec3( vuf * Inv255, gf * PackUpscale, bf );
}
vec2 packDepthToRG( const in float v ) {
	if( v <= 0.0 )
		return vec2( 0., 0. );
	if( v >= 1.0 )
		return vec2( 1., 1. );
	float vuf;
	float gf = modf( v * 256., vuf );
	return vec2( vuf * Inv255, gf );
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors4 );
}
float unpackRGBToDepth( const in vec3 v ) {
	return dot( v, UnpackFactors3 );
}
float unpackRGToDepth( const in vec2 v ) {
	return v.r * UnpackFactors2.r + v.g * UnpackFactors2.g;
}
vec4 pack2HalfToRGBA( const in vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( const in vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,aS=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,oS=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,lS=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,uS=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,cS=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,fS=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,dS=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,hS=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,pS=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,mS=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,gS=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,_S=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,vS=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,xS=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,yS=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,SS=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,MS=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,ES=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 OptimizedCineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,wS=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,TS=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
		
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
		
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		
		#else
		
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,AS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,CS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,RS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,bS=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const PS=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,LS=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,NS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,DS=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,IS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,US=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,FS=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,OS=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#elif DEPTH_PACKING == 3202
		gl_FragColor = vec4( packDepthToRGB( fragCoordZ ), 1.0 );
	#elif DEPTH_PACKING == 3203
		gl_FragColor = vec4( packDepthToRG( fragCoordZ ), 0.0, 1.0 );
	#endif
}`,kS=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,BS=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,zS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,HS=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,VS=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,GS=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,WS=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,jS=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,XS=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,YS=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,qS=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,$S=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,KS=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,ZS=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,QS=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,JS=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,eM=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,tM=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,nM=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,iM=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,rM=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,sM=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,aM=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,oM=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,lM=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix * vec4( 0.0, 0.0, 0.0, 1.0 );
	vec2 scale;
	scale.x = length( vec3( modelMatrix[ 0 ].x, modelMatrix[ 0 ].y, modelMatrix[ 0 ].z ) );
	scale.y = length( vec3( modelMatrix[ 1 ].x, modelMatrix[ 1 ].y, modelMatrix[ 1 ].z ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,uM=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,pt={alphahash_fragment:Lx,alphahash_pars_fragment:Nx,alphamap_fragment:Dx,alphamap_pars_fragment:Ix,alphatest_fragment:Ux,alphatest_pars_fragment:Fx,aomap_fragment:Ox,aomap_pars_fragment:kx,batching_pars_vertex:Bx,batching_vertex:zx,begin_vertex:Hx,beginnormal_vertex:Vx,bsdfs:Gx,iridescence_fragment:Wx,bumpmap_pars_fragment:jx,clipping_planes_fragment:Xx,clipping_planes_pars_fragment:Yx,clipping_planes_pars_vertex:qx,clipping_planes_vertex:$x,color_fragment:Kx,color_pars_fragment:Zx,color_pars_vertex:Qx,color_vertex:Jx,common:ey,cube_uv_reflection_fragment:ty,defaultnormal_vertex:ny,displacementmap_pars_vertex:iy,displacementmap_vertex:ry,emissivemap_fragment:sy,emissivemap_pars_fragment:ay,colorspace_fragment:oy,colorspace_pars_fragment:ly,envmap_fragment:uy,envmap_common_pars_fragment:cy,envmap_pars_fragment:fy,envmap_pars_vertex:dy,envmap_physical_pars_fragment:Ey,envmap_vertex:hy,fog_vertex:py,fog_pars_vertex:my,fog_fragment:gy,fog_pars_fragment:_y,gradientmap_pars_fragment:vy,lightmap_pars_fragment:xy,lights_lambert_fragment:yy,lights_lambert_pars_fragment:Sy,lights_pars_begin:My,lights_toon_fragment:wy,lights_toon_pars_fragment:Ty,lights_phong_fragment:Ay,lights_phong_pars_fragment:Cy,lights_physical_fragment:Ry,lights_physical_pars_fragment:by,lights_fragment_begin:Py,lights_fragment_maps:Ly,lights_fragment_end:Ny,logdepthbuf_fragment:Dy,logdepthbuf_pars_fragment:Iy,logdepthbuf_pars_vertex:Uy,logdepthbuf_vertex:Fy,map_fragment:Oy,map_pars_fragment:ky,map_particle_fragment:By,map_particle_pars_fragment:zy,metalnessmap_fragment:Hy,metalnessmap_pars_fragment:Vy,morphinstance_vertex:Gy,morphcolor_vertex:Wy,morphnormal_vertex:jy,morphtarget_pars_vertex:Xy,morphtarget_vertex:Yy,normal_fragment_begin:qy,normal_fragment_maps:$y,normal_pars_fragment:Ky,normal_pars_vertex:Zy,normal_vertex:Qy,normalmap_pars_fragment:Jy,clearcoat_normal_fragment_begin:eS,clearcoat_normal_fragment_maps:tS,clearcoat_pars_fragment:nS,iridescence_pars_fragment:iS,opaque_fragment:rS,packing:sS,premultiplied_alpha_fragment:aS,project_vertex:oS,dithering_fragment:lS,dithering_pars_fragment:uS,roughnessmap_fragment:cS,roughnessmap_pars_fragment:fS,shadowmap_pars_fragment:dS,shadowmap_pars_vertex:hS,shadowmap_vertex:pS,shadowmask_pars_fragment:mS,skinbase_vertex:gS,skinning_pars_vertex:_S,skinning_vertex:vS,skinnormal_vertex:xS,specularmap_fragment:yS,specularmap_pars_fragment:SS,tonemapping_fragment:MS,tonemapping_pars_fragment:ES,transmission_fragment:wS,transmission_pars_fragment:TS,uv_pars_fragment:AS,uv_pars_vertex:CS,uv_vertex:RS,worldpos_vertex:bS,background_vert:PS,background_frag:LS,backgroundCube_vert:NS,backgroundCube_frag:DS,cube_vert:IS,cube_frag:US,depth_vert:FS,depth_frag:OS,distanceRGBA_vert:kS,distanceRGBA_frag:BS,equirect_vert:zS,equirect_frag:HS,linedashed_vert:VS,linedashed_frag:GS,meshbasic_vert:WS,meshbasic_frag:jS,meshlambert_vert:XS,meshlambert_frag:YS,meshmatcap_vert:qS,meshmatcap_frag:$S,meshnormal_vert:KS,meshnormal_frag:ZS,meshphong_vert:QS,meshphong_frag:JS,meshphysical_vert:eM,meshphysical_frag:tM,meshtoon_vert:nM,meshtoon_frag:iM,points_vert:rM,points_frag:sM,shadow_vert:aM,shadow_frag:oM,sprite_vert:lM,sprite_frag:uM},Fe={common:{diffuse:{value:new yt(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new mt},alphaMap:{value:null},alphaMapTransform:{value:new mt},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new mt}},envmap:{envMap:{value:null},envMapRotation:{value:new mt},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new mt}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new mt}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new mt},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new mt},normalScale:{value:new ht(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new mt},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new mt}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new mt}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new mt}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new yt(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new yt(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new mt},alphaTest:{value:0},uvTransform:{value:new mt}},sprite:{diffuse:{value:new yt(16777215)},opacity:{value:1},center:{value:new ht(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new mt},alphaMap:{value:null},alphaMapTransform:{value:new mt},alphaTest:{value:0}}},vi={basic:{uniforms:yn([Fe.common,Fe.specularmap,Fe.envmap,Fe.aomap,Fe.lightmap,Fe.fog]),vertexShader:pt.meshbasic_vert,fragmentShader:pt.meshbasic_frag},lambert:{uniforms:yn([Fe.common,Fe.specularmap,Fe.envmap,Fe.aomap,Fe.lightmap,Fe.emissivemap,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,Fe.fog,Fe.lights,{emissive:{value:new yt(0)}}]),vertexShader:pt.meshlambert_vert,fragmentShader:pt.meshlambert_frag},phong:{uniforms:yn([Fe.common,Fe.specularmap,Fe.envmap,Fe.aomap,Fe.lightmap,Fe.emissivemap,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,Fe.fog,Fe.lights,{emissive:{value:new yt(0)},specular:{value:new yt(1118481)},shininess:{value:30}}]),vertexShader:pt.meshphong_vert,fragmentShader:pt.meshphong_frag},standard:{uniforms:yn([Fe.common,Fe.envmap,Fe.aomap,Fe.lightmap,Fe.emissivemap,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,Fe.roughnessmap,Fe.metalnessmap,Fe.fog,Fe.lights,{emissive:{value:new yt(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:pt.meshphysical_vert,fragmentShader:pt.meshphysical_frag},toon:{uniforms:yn([Fe.common,Fe.aomap,Fe.lightmap,Fe.emissivemap,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,Fe.gradientmap,Fe.fog,Fe.lights,{emissive:{value:new yt(0)}}]),vertexShader:pt.meshtoon_vert,fragmentShader:pt.meshtoon_frag},matcap:{uniforms:yn([Fe.common,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,Fe.fog,{matcap:{value:null}}]),vertexShader:pt.meshmatcap_vert,fragmentShader:pt.meshmatcap_frag},points:{uniforms:yn([Fe.points,Fe.fog]),vertexShader:pt.points_vert,fragmentShader:pt.points_frag},dashed:{uniforms:yn([Fe.common,Fe.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:pt.linedashed_vert,fragmentShader:pt.linedashed_frag},depth:{uniforms:yn([Fe.common,Fe.displacementmap]),vertexShader:pt.depth_vert,fragmentShader:pt.depth_frag},normal:{uniforms:yn([Fe.common,Fe.bumpmap,Fe.normalmap,Fe.displacementmap,{opacity:{value:1}}]),vertexShader:pt.meshnormal_vert,fragmentShader:pt.meshnormal_frag},sprite:{uniforms:yn([Fe.sprite,Fe.fog]),vertexShader:pt.sprite_vert,fragmentShader:pt.sprite_frag},background:{uniforms:{uvTransform:{value:new mt},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:pt.background_vert,fragmentShader:pt.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new mt}},vertexShader:pt.backgroundCube_vert,fragmentShader:pt.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:pt.cube_vert,fragmentShader:pt.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:pt.equirect_vert,fragmentShader:pt.equirect_frag},distanceRGBA:{uniforms:yn([Fe.common,Fe.displacementmap,{referencePosition:{value:new Q},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:pt.distanceRGBA_vert,fragmentShader:pt.distanceRGBA_frag},shadow:{uniforms:yn([Fe.lights,Fe.fog,{color:{value:new yt(0)},opacity:{value:1}}]),vertexShader:pt.shadow_vert,fragmentShader:pt.shadow_frag}};vi.physical={uniforms:yn([vi.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new mt},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new mt},clearcoatNormalScale:{value:new ht(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new mt},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new mt},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new mt},sheen:{value:0},sheenColor:{value:new yt(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new mt},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new mt},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new mt},transmissionSamplerSize:{value:new ht},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new mt},attenuationDistance:{value:0},attenuationColor:{value:new yt(0)},specularColor:{value:new yt(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new mt},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new mt},anisotropyVector:{value:new ht},anisotropyMap:{value:null},anisotropyMapTransform:{value:new mt}}]),vertexShader:pt.meshphysical_vert,fragmentShader:pt.meshphysical_frag};const Dl={r:0,b:0,g:0},Gr=new Si,cM=new Gt;function fM(s,e,t,r,o,u,c){const f=new yt(0);let h=u===!0?0:1,m,g,y=null,v=0,M=null;function T(P){let R=P.isScene===!0?P.background:null;return R&&R.isTexture&&(R=(P.backgroundBlurriness>0?t:e).get(R)),R}function S(P){let R=!1;const L=T(P);L===null?_(f,h):L&&L.isColor&&(_(L,1),R=!0);const Y=s.xr.getEnvironmentBlendMode();Y==="additive"?r.buffers.color.setClear(0,0,0,1,c):Y==="alpha-blend"&&r.buffers.color.setClear(0,0,0,0,c),(s.autoClear||R)&&(r.buffers.depth.setTest(!0),r.buffers.depth.setMask(!0),r.buffers.color.setMask(!0),s.clear(s.autoClearColor,s.autoClearDepth,s.autoClearStencil))}function x(P,R){const L=T(R);L&&(L.isCubeTexture||L.mapping===cu)?(g===void 0&&(g=new zn(new ns(1,1,1),new Er({name:"BackgroundCubeMaterial",uniforms:ua(vi.backgroundCube.uniforms),vertexShader:vi.backgroundCube.vertexShader,fragmentShader:vi.backgroundCube.fragmentShader,side:Ln,depthTest:!1,depthWrite:!1,fog:!1})),g.geometry.deleteAttribute("normal"),g.geometry.deleteAttribute("uv"),g.onBeforeRender=function(Y,U,N){this.matrixWorld.copyPosition(N.matrixWorld)},Object.defineProperty(g.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),o.update(g)),Gr.copy(R.backgroundRotation),Gr.x*=-1,Gr.y*=-1,Gr.z*=-1,L.isCubeTexture&&L.isRenderTargetTexture===!1&&(Gr.y*=-1,Gr.z*=-1),g.material.uniforms.envMap.value=L,g.material.uniforms.flipEnvMap.value=L.isCubeTexture&&L.isRenderTargetTexture===!1?-1:1,g.material.uniforms.backgroundBlurriness.value=R.backgroundBlurriness,g.material.uniforms.backgroundIntensity.value=R.backgroundIntensity,g.material.uniforms.backgroundRotation.value.setFromMatrix4(cM.makeRotationFromEuler(Gr)),g.material.toneMapped=Rt.getTransfer(L.colorSpace)!==Ot,(y!==L||v!==L.version||M!==s.toneMapping)&&(g.material.needsUpdate=!0,y=L,v=L.version,M=s.toneMapping),g.layers.enableAll(),P.unshift(g,g.geometry,g.material,0,0,null)):L&&L.isTexture&&(m===void 0&&(m=new zn(new hu(2,2),new Er({name:"BackgroundMaterial",uniforms:ua(vi.background.uniforms),vertexShader:vi.background.vertexShader,fragmentShader:vi.background.fragmentShader,side:Mr,depthTest:!1,depthWrite:!1,fog:!1})),m.geometry.deleteAttribute("normal"),Object.defineProperty(m.material,"map",{get:function(){return this.uniforms.t2D.value}}),o.update(m)),m.material.uniforms.t2D.value=L,m.material.uniforms.backgroundIntensity.value=R.backgroundIntensity,m.material.toneMapped=Rt.getTransfer(L.colorSpace)!==Ot,L.matrixAutoUpdate===!0&&L.updateMatrix(),m.material.uniforms.uvTransform.value.copy(L.matrix),(y!==L||v!==L.version||M!==s.toneMapping)&&(m.material.needsUpdate=!0,y=L,v=L.version,M=s.toneMapping),m.layers.enableAll(),P.unshift(m,m.geometry,m.material,0,0,null))}function _(P,R){P.getRGB(Dl,n_(s)),r.buffers.color.setClear(Dl.r,Dl.g,Dl.b,R,c)}return{getClearColor:function(){return f},setClearColor:function(P,R=1){f.set(P),h=R,_(f,h)},getClearAlpha:function(){return h},setClearAlpha:function(P){h=P,_(f,h)},render:S,addToRenderList:x}}function dM(s,e){const t=s.getParameter(s.MAX_VERTEX_ATTRIBS),r={},o=v(null);let u=o,c=!1;function f(w,I,K,V,Z){let te=!1;const ue=y(V,K,I);u!==ue&&(u=ue,m(u.object)),te=M(w,V,K,Z),te&&T(w,V,K,Z),Z!==null&&e.update(Z,s.ELEMENT_ARRAY_BUFFER),(te||c)&&(c=!1,L(w,I,K,V),Z!==null&&s.bindBuffer(s.ELEMENT_ARRAY_BUFFER,e.get(Z).buffer))}function h(){return s.createVertexArray()}function m(w){return s.bindVertexArray(w)}function g(w){return s.deleteVertexArray(w)}function y(w,I,K){const V=K.wireframe===!0;let Z=r[w.id];Z===void 0&&(Z={},r[w.id]=Z);let te=Z[I.id];te===void 0&&(te={},Z[I.id]=te);let ue=te[V];return ue===void 0&&(ue=v(h()),te[V]=ue),ue}function v(w){const I=[],K=[],V=[];for(let Z=0;Z<t;Z++)I[Z]=0,K[Z]=0,V[Z]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:I,enabledAttributes:K,attributeDivisors:V,object:w,attributes:{},index:null}}function M(w,I,K,V){const Z=u.attributes,te=I.attributes;let ue=0;const se=K.getAttributes();for(const G in se)if(se[G].location>=0){const fe=Z[G];let k=te[G];if(k===void 0&&(G==="instanceMatrix"&&w.instanceMatrix&&(k=w.instanceMatrix),G==="instanceColor"&&w.instanceColor&&(k=w.instanceColor)),fe===void 0||fe.attribute!==k||k&&fe.data!==k.data)return!0;ue++}return u.attributesNum!==ue||u.index!==V}function T(w,I,K,V){const Z={},te=I.attributes;let ue=0;const se=K.getAttributes();for(const G in se)if(se[G].location>=0){let fe=te[G];fe===void 0&&(G==="instanceMatrix"&&w.instanceMatrix&&(fe=w.instanceMatrix),G==="instanceColor"&&w.instanceColor&&(fe=w.instanceColor));const k={};k.attribute=fe,fe&&fe.data&&(k.data=fe.data),Z[G]=k,ue++}u.attributes=Z,u.attributesNum=ue,u.index=V}function S(){const w=u.newAttributes;for(let I=0,K=w.length;I<K;I++)w[I]=0}function x(w){_(w,0)}function _(w,I){const K=u.newAttributes,V=u.enabledAttributes,Z=u.attributeDivisors;K[w]=1,V[w]===0&&(s.enableVertexAttribArray(w),V[w]=1),Z[w]!==I&&(s.vertexAttribDivisor(w,I),Z[w]=I)}function P(){const w=u.newAttributes,I=u.enabledAttributes;for(let K=0,V=I.length;K<V;K++)I[K]!==w[K]&&(s.disableVertexAttribArray(K),I[K]=0)}function R(w,I,K,V,Z,te,ue){ue===!0?s.vertexAttribIPointer(w,I,K,Z,te):s.vertexAttribPointer(w,I,K,V,Z,te)}function L(w,I,K,V){S();const Z=V.attributes,te=K.getAttributes(),ue=I.defaultAttributeValues;for(const se in te){const G=te[se];if(G.location>=0){let he=Z[se];if(he===void 0&&(se==="instanceMatrix"&&w.instanceMatrix&&(he=w.instanceMatrix),se==="instanceColor"&&w.instanceColor&&(he=w.instanceColor)),he!==void 0){const fe=he.normalized,k=he.itemSize,ce=e.get(he);if(ce===void 0)continue;const ke=ce.buffer,J=ce.type,de=ce.bytesPerElement,Se=J===s.INT||J===s.UNSIGNED_INT||he.gpuType===Ed;if(he.isInterleavedBufferAttribute){const Me=he.data,Ie=Me.stride,Ge=he.offset;if(Me.isInstancedInterleavedBuffer){for(let Qe=0;Qe<G.locationSize;Qe++)_(G.location+Qe,Me.meshPerAttribute);w.isInstancedMesh!==!0&&V._maxInstanceCount===void 0&&(V._maxInstanceCount=Me.meshPerAttribute*Me.count)}else for(let Qe=0;Qe<G.locationSize;Qe++)x(G.location+Qe);s.bindBuffer(s.ARRAY_BUFFER,ke);for(let Qe=0;Qe<G.locationSize;Qe++)R(G.location+Qe,k/G.locationSize,J,fe,Ie*de,(Ge+k/G.locationSize*Qe)*de,Se)}else{if(he.isInstancedBufferAttribute){for(let Me=0;Me<G.locationSize;Me++)_(G.location+Me,he.meshPerAttribute);w.isInstancedMesh!==!0&&V._maxInstanceCount===void 0&&(V._maxInstanceCount=he.meshPerAttribute*he.count)}else for(let Me=0;Me<G.locationSize;Me++)x(G.location+Me);s.bindBuffer(s.ARRAY_BUFFER,ke);for(let Me=0;Me<G.locationSize;Me++)R(G.location+Me,k/G.locationSize,J,fe,k*de,k/G.locationSize*Me*de,Se)}}else if(ue!==void 0){const fe=ue[se];if(fe!==void 0)switch(fe.length){case 2:s.vertexAttrib2fv(G.location,fe);break;case 3:s.vertexAttrib3fv(G.location,fe);break;case 4:s.vertexAttrib4fv(G.location,fe);break;default:s.vertexAttrib1fv(G.location,fe)}}}}P()}function Y(){H();for(const w in r){const I=r[w];for(const K in I){const V=I[K];for(const Z in V)g(V[Z].object),delete V[Z];delete I[K]}delete r[w]}}function U(w){if(r[w.id]===void 0)return;const I=r[w.id];for(const K in I){const V=I[K];for(const Z in V)g(V[Z].object),delete V[Z];delete I[K]}delete r[w.id]}function N(w){for(const I in r){const K=r[I];if(K[w.id]===void 0)continue;const V=K[w.id];for(const Z in V)g(V[Z].object),delete V[Z];delete K[w.id]}}function H(){b(),c=!0,u!==o&&(u=o,m(u.object))}function b(){o.geometry=null,o.program=null,o.wireframe=!1}return{setup:f,reset:H,resetDefaultState:b,dispose:Y,releaseStatesOfGeometry:U,releaseStatesOfProgram:N,initAttributes:S,enableAttribute:x,disableUnusedAttributes:P}}function hM(s,e,t){let r;function o(m){r=m}function u(m,g){s.drawArrays(r,m,g),t.update(g,r,1)}function c(m,g,y){y!==0&&(s.drawArraysInstanced(r,m,g,y),t.update(g,r,y))}function f(m,g,y){if(y===0)return;e.get("WEBGL_multi_draw").multiDrawArraysWEBGL(r,m,0,g,0,y);let M=0;for(let T=0;T<y;T++)M+=g[T];t.update(M,r,1)}function h(m,g,y,v){if(y===0)return;const M=e.get("WEBGL_multi_draw");if(M===null)for(let T=0;T<m.length;T++)c(m[T],g[T],v[T]);else{M.multiDrawArraysInstancedWEBGL(r,m,0,g,0,v,0,y);let T=0;for(let S=0;S<y;S++)T+=g[S];for(let S=0;S<v.length;S++)t.update(T,r,v[S])}}this.setMode=o,this.render=u,this.renderInstances=c,this.renderMultiDraw=f,this.renderMultiDrawInstances=h}function pM(s,e,t,r){let o;function u(){if(o!==void 0)return o;if(e.has("EXT_texture_filter_anisotropic")===!0){const U=e.get("EXT_texture_filter_anisotropic");o=s.getParameter(U.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else o=0;return o}function c(U){return!(U!==di&&r.convert(U)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_FORMAT))}function f(U){const N=U===ro&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(U!==Vi&&r.convert(U)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_TYPE)&&U!==zi&&!N)}function h(U){if(U==="highp"){if(s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.HIGH_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.HIGH_FLOAT).precision>0)return"highp";U="mediump"}return U==="mediump"&&s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.MEDIUM_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let m=t.precision!==void 0?t.precision:"highp";const g=h(m);g!==m&&(console.warn("THREE.WebGLRenderer:",m,"not supported, using",g,"instead."),m=g);const y=t.logarithmicDepthBuffer===!0,v=s.getParameter(s.MAX_TEXTURE_IMAGE_UNITS),M=s.getParameter(s.MAX_VERTEX_TEXTURE_IMAGE_UNITS),T=s.getParameter(s.MAX_TEXTURE_SIZE),S=s.getParameter(s.MAX_CUBE_MAP_TEXTURE_SIZE),x=s.getParameter(s.MAX_VERTEX_ATTRIBS),_=s.getParameter(s.MAX_VERTEX_UNIFORM_VECTORS),P=s.getParameter(s.MAX_VARYING_VECTORS),R=s.getParameter(s.MAX_FRAGMENT_UNIFORM_VECTORS),L=M>0,Y=s.getParameter(s.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:u,getMaxPrecision:h,textureFormatReadable:c,textureTypeReadable:f,precision:m,logarithmicDepthBuffer:y,maxTextures:v,maxVertexTextures:M,maxTextureSize:T,maxCubemapSize:S,maxAttributes:x,maxVertexUniforms:_,maxVaryings:P,maxFragmentUniforms:R,vertexTextures:L,maxSamples:Y}}function mM(s){const e=this;let t=null,r=0,o=!1,u=!1;const c=new jr,f=new mt,h={value:null,needsUpdate:!1};this.uniform=h,this.numPlanes=0,this.numIntersection=0,this.init=function(y,v){const M=y.length!==0||v||r!==0||o;return o=v,r=y.length,M},this.beginShadows=function(){u=!0,g(null)},this.endShadows=function(){u=!1},this.setGlobalState=function(y,v){t=g(y,v,0)},this.setState=function(y,v,M){const T=y.clippingPlanes,S=y.clipIntersection,x=y.clipShadows,_=s.get(y);if(!o||T===null||T.length===0||u&&!x)u?g(null):m();else{const P=u?0:r,R=P*4;let L=_.clippingState||null;h.value=L,L=g(T,v,R,M);for(let Y=0;Y!==R;++Y)L[Y]=t[Y];_.clippingState=L,this.numIntersection=S?this.numPlanes:0,this.numPlanes+=P}};function m(){h.value!==t&&(h.value=t,h.needsUpdate=r>0),e.numPlanes=r,e.numIntersection=0}function g(y,v,M,T){const S=y!==null?y.length:0;let x=null;if(S!==0){if(x=h.value,T!==!0||x===null){const _=M+S*4,P=v.matrixWorldInverse;f.getNormalMatrix(P),(x===null||x.length<_)&&(x=new Float32Array(_));for(let R=0,L=M;R!==S;++R,L+=4)c.copy(y[R]).applyMatrix4(P,f),c.normal.toArray(x,L),x[L+3]=c.constant}h.value=x,h.needsUpdate=!0}return e.numPlanes=S,e.numIntersection=0,x}}function gM(s){let e=new WeakMap;function t(c,f){return f===Vf?c.mapping=sa:f===Gf&&(c.mapping=aa),c}function r(c){if(c&&c.isTexture){const f=c.mapping;if(f===Vf||f===Gf)if(e.has(c)){const h=e.get(c).texture;return t(h,c.mapping)}else{const h=c.image;if(h&&h.height>0){const m=new Cx(h.height);return m.fromEquirectangularTexture(s,c),e.set(c,m),c.addEventListener("dispose",o),t(m.texture,c.mapping)}else return null}}return c}function o(c){const f=c.target;f.removeEventListener("dispose",o);const h=e.get(f);h!==void 0&&(e.delete(f),h.dispose())}function u(){e=new WeakMap}return{get:r,dispose:u}}class Ks extends i_{constructor(e=-1,t=1,r=1,o=-1,u=.1,c=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=r,this.bottom=o,this.near=u,this.far=c,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,r,o,u,c){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=r,this.view.offsetY=o,this.view.width=u,this.view.height=c,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),r=(this.right+this.left)/2,o=(this.top+this.bottom)/2;let u=r-e,c=r+e,f=o+t,h=o-t;if(this.view!==null&&this.view.enabled){const m=(this.right-this.left)/this.view.fullWidth/this.zoom,g=(this.top-this.bottom)/this.view.fullHeight/this.zoom;u+=m*this.view.offsetX,c=u+m*this.view.width,f-=g*this.view.offsetY,h=f-g*this.view.height}this.projectionMatrix.makeOrthographic(u,c,f,h,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}}const Zs=4,Gm=[.125,.215,.35,.446,.526,.582],$r=20,Mf=new Ks,Wm=new yt;let Ef=null,wf=0,Tf=0,Af=!1;const Xr=(1+Math.sqrt(5))/2,Gs=1/Xr,jm=[new Q(-Xr,Gs,0),new Q(Xr,Gs,0),new Q(-Gs,0,Xr),new Q(Gs,0,Xr),new Q(0,Xr,-Gs),new Q(0,Xr,Gs),new Q(-1,1,-1),new Q(1,1,-1),new Q(-1,1,1),new Q(1,1,1)];class Xm{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,r=.1,o=100){Ef=this._renderer.getRenderTarget(),wf=this._renderer.getActiveCubeFace(),Tf=this._renderer.getActiveMipmapLevel(),Af=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);const u=this._allocateTargets();return u.depthBuffer=!0,this._sceneToCubeUV(e,r,o,u),t>0&&this._blur(u,0,0,t),this._applyPMREM(u),this._cleanup(u),u}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=$m(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=qm(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget(Ef,wf,Tf),this._renderer.xr.enabled=Af,e.scissorTest=!1,Il(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===sa||e.mapping===aa?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),Ef=this._renderer.getRenderTarget(),wf=this._renderer.getActiveCubeFace(),Tf=this._renderer.getActiveMipmapLevel(),Af=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const r=t||this._allocateTargets();return this._textureToCubeUV(e,r),this._applyPMREM(r),this._cleanup(r),r}_allocateTargets(){const e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,r={magFilter:ci,minFilter:ci,generateMipmaps:!1,type:ro,format:di,colorSpace:wr,depthBuffer:!1},o=Ym(e,t,r);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=Ym(e,t,r);const{_lodMax:u}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=_M(u)),this._blurMaterial=vM(u,e,t)}return o}_compileMaterial(e){const t=new zn(this._lodPlanes[0],e);this._renderer.compile(t,Mf)}_sceneToCubeUV(e,t,r,o){const f=new $n(90,1,t,r),h=[1,-1,1,1,1,1],m=[1,1,1,-1,-1,-1],g=this._renderer,y=g.autoClear,v=g.toneMapping;g.getClearColor(Wm),g.toneMapping=yr,g.autoClear=!1;const M=new Pd({name:"PMREM.Background",side:Ln,depthWrite:!1,depthTest:!1}),T=new zn(new ns,M);let S=!1;const x=e.background;x?x.isColor&&(M.color.copy(x),e.background=null,S=!0):(M.color.copy(Wm),S=!0);for(let _=0;_<6;_++){const P=_%3;P===0?(f.up.set(0,h[_],0),f.lookAt(m[_],0,0)):P===1?(f.up.set(0,0,h[_]),f.lookAt(0,m[_],0)):(f.up.set(0,h[_],0),f.lookAt(0,0,m[_]));const R=this._cubeSize;Il(o,P*R,_>2?R:0,R,R),g.setRenderTarget(o),S&&g.render(T,f),g.render(e,f)}T.geometry.dispose(),T.material.dispose(),g.toneMapping=v,g.autoClear=y,e.background=x}_textureToCubeUV(e,t){const r=this._renderer,o=e.mapping===sa||e.mapping===aa;o?(this._cubemapMaterial===null&&(this._cubemapMaterial=$m()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=qm());const u=o?this._cubemapMaterial:this._equirectMaterial,c=new zn(this._lodPlanes[0],u),f=u.uniforms;f.envMap.value=e;const h=this._cubeSize;Il(t,0,0,3*h,2*h),r.setRenderTarget(t),r.render(c,Mf)}_applyPMREM(e){const t=this._renderer,r=t.autoClear;t.autoClear=!1;const o=this._lodPlanes.length;for(let u=1;u<o;u++){const c=Math.sqrt(this._sigmas[u]*this._sigmas[u]-this._sigmas[u-1]*this._sigmas[u-1]),f=jm[(o-u-1)%jm.length];this._blur(e,u-1,u,c,f)}t.autoClear=r}_blur(e,t,r,o,u){const c=this._pingPongRenderTarget;this._halfBlur(e,c,t,r,o,"latitudinal",u),this._halfBlur(c,e,r,r,o,"longitudinal",u)}_halfBlur(e,t,r,o,u,c,f){const h=this._renderer,m=this._blurMaterial;c!=="latitudinal"&&c!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const g=3,y=new zn(this._lodPlanes[o],m),v=m.uniforms,M=this._sizeLods[r]-1,T=isFinite(u)?Math.PI/(2*M):2*Math.PI/(2*$r-1),S=u/T,x=isFinite(u)?1+Math.floor(g*S):$r;x>$r&&console.warn(`sigmaRadians, ${u}, is too large and will clip, as it requested ${x} samples when the maximum is set to ${$r}`);const _=[];let P=0;for(let N=0;N<$r;++N){const H=N/S,b=Math.exp(-H*H/2);_.push(b),N===0?P+=b:N<x&&(P+=2*b)}for(let N=0;N<_.length;N++)_[N]=_[N]/P;v.envMap.value=e.texture,v.samples.value=x,v.weights.value=_,v.latitudinal.value=c==="latitudinal",f&&(v.poleAxis.value=f);const{_lodMax:R}=this;v.dTheta.value=T,v.mipInt.value=R-r;const L=this._sizeLods[o],Y=3*L*(o>R-Zs?o-R+Zs:0),U=4*(this._cubeSize-L);Il(t,Y,U,3*L,2*L),h.setRenderTarget(t),h.render(y,Mf)}}function _M(s){const e=[],t=[],r=[];let o=s;const u=s-Zs+1+Gm.length;for(let c=0;c<u;c++){const f=Math.pow(2,o);t.push(f);let h=1/f;c>s-Zs?h=Gm[c-s+Zs-1]:c===0&&(h=0),r.push(h);const m=1/(f-2),g=-m,y=1+m,v=[g,g,y,g,y,y,g,g,y,y,g,y],M=6,T=6,S=3,x=2,_=1,P=new Float32Array(S*T*M),R=new Float32Array(x*T*M),L=new Float32Array(_*T*M);for(let U=0;U<M;U++){const N=U%3*2/3-1,H=U>2?0:-1,b=[N,H,0,N+2/3,H,0,N+2/3,H+1,0,N,H,0,N+2/3,H+1,0,N,H+1,0];P.set(b,S*T*U),R.set(v,x*T*U);const w=[U,U,U,U,U,U];L.set(w,_*T*U)}const Y=new Hn;Y.setAttribute("position",new hi(P,S)),Y.setAttribute("uv",new hi(R,x)),Y.setAttribute("faceIndex",new hi(L,_)),e.push(Y),o>Zs&&o--}return{lodPlanes:e,sizeLods:t,sigmas:r}}function Ym(s,e,t){const r=new ts(s,e,t);return r.texture.mapping=cu,r.texture.name="PMREM.cubeUv",r.scissorTest=!0,r}function Il(s,e,t,r,o){s.viewport.set(e,t,r,o),s.scissor.set(e,t,r,o)}function vM(s,e,t){const r=new Float32Array($r),o=new Q(0,1,0);return new Er({name:"SphericalGaussianBlur",defines:{n:$r,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${s}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:r},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:o}},vertexShader:Nd(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:xr,depthTest:!1,depthWrite:!1})}function qm(){return new Er({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Nd(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:xr,depthTest:!1,depthWrite:!1})}function $m(){return new Er({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Nd(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:xr,depthTest:!1,depthWrite:!1})}function Nd(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function xM(s){let e=new WeakMap,t=null;function r(f){if(f&&f.isTexture){const h=f.mapping,m=h===Vf||h===Gf,g=h===sa||h===aa;if(m||g){let y=e.get(f);const v=y!==void 0?y.texture.pmremVersion:0;if(f.isRenderTargetTexture&&f.pmremVersion!==v)return t===null&&(t=new Xm(s)),y=m?t.fromEquirectangular(f,y):t.fromCubemap(f,y),y.texture.pmremVersion=f.pmremVersion,e.set(f,y),y.texture;if(y!==void 0)return y.texture;{const M=f.image;return m&&M&&M.height>0||g&&M&&o(M)?(t===null&&(t=new Xm(s)),y=m?t.fromEquirectangular(f):t.fromCubemap(f),y.texture.pmremVersion=f.pmremVersion,e.set(f,y),f.addEventListener("dispose",u),y.texture):null}}}return f}function o(f){let h=0;const m=6;for(let g=0;g<m;g++)f[g]!==void 0&&h++;return h===m}function u(f){const h=f.target;h.removeEventListener("dispose",u);const m=e.get(h);m!==void 0&&(e.delete(h),m.dispose())}function c(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:r,dispose:c}}function yM(s){const e={};function t(r){if(e[r]!==void 0)return e[r];let o;switch(r){case"WEBGL_depth_texture":o=s.getExtension("WEBGL_depth_texture")||s.getExtension("MOZ_WEBGL_depth_texture")||s.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":o=s.getExtension("EXT_texture_filter_anisotropic")||s.getExtension("MOZ_EXT_texture_filter_anisotropic")||s.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":o=s.getExtension("WEBGL_compressed_texture_s3tc")||s.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":o=s.getExtension("WEBGL_compressed_texture_pvrtc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:o=s.getExtension(r)}return e[r]=o,o}return{has:function(r){return t(r)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(r){const o=t(r);return o===null&&ta("THREE.WebGLRenderer: "+r+" extension not supported."),o}}}function SM(s,e,t,r){const o={},u=new WeakMap;function c(y){const v=y.target;v.index!==null&&e.remove(v.index);for(const T in v.attributes)e.remove(v.attributes[T]);for(const T in v.morphAttributes){const S=v.morphAttributes[T];for(let x=0,_=S.length;x<_;x++)e.remove(S[x])}v.removeEventListener("dispose",c),delete o[v.id];const M=u.get(v);M&&(e.remove(M),u.delete(v)),r.releaseStatesOfGeometry(v),v.isInstancedBufferGeometry===!0&&delete v._maxInstanceCount,t.memory.geometries--}function f(y,v){return o[v.id]===!0||(v.addEventListener("dispose",c),o[v.id]=!0,t.memory.geometries++),v}function h(y){const v=y.attributes;for(const T in v)e.update(v[T],s.ARRAY_BUFFER);const M=y.morphAttributes;for(const T in M){const S=M[T];for(let x=0,_=S.length;x<_;x++)e.update(S[x],s.ARRAY_BUFFER)}}function m(y){const v=[],M=y.index,T=y.attributes.position;let S=0;if(M!==null){const P=M.array;S=M.version;for(let R=0,L=P.length;R<L;R+=3){const Y=P[R+0],U=P[R+1],N=P[R+2];v.push(Y,U,U,N,N,Y)}}else if(T!==void 0){const P=T.array;S=T.version;for(let R=0,L=P.length/3-1;R<L;R+=3){const Y=R+0,U=R+1,N=R+2;v.push(Y,U,U,N,N,Y)}}else return;const x=new(qg(v)?t_:e_)(v,1);x.version=S;const _=u.get(y);_&&e.remove(_),u.set(y,x)}function g(y){const v=u.get(y);if(v){const M=y.index;M!==null&&v.version<M.version&&m(y)}else m(y);return u.get(y)}return{get:f,update:h,getWireframeAttribute:g}}function MM(s,e,t){let r;function o(v){r=v}let u,c;function f(v){u=v.type,c=v.bytesPerElement}function h(v,M){s.drawElements(r,M,u,v*c),t.update(M,r,1)}function m(v,M,T){T!==0&&(s.drawElementsInstanced(r,M,u,v*c,T),t.update(M,r,T))}function g(v,M,T){if(T===0)return;e.get("WEBGL_multi_draw").multiDrawElementsWEBGL(r,M,0,u,v,0,T);let x=0;for(let _=0;_<T;_++)x+=M[_];t.update(x,r,1)}function y(v,M,T,S){if(T===0)return;const x=e.get("WEBGL_multi_draw");if(x===null)for(let _=0;_<v.length;_++)m(v[_]/c,M[_],S[_]);else{x.multiDrawElementsInstancedWEBGL(r,M,0,u,v,0,S,0,T);let _=0;for(let P=0;P<T;P++)_+=M[P];for(let P=0;P<S.length;P++)t.update(_,r,S[P])}}this.setMode=o,this.setIndex=f,this.render=h,this.renderInstances=m,this.renderMultiDraw=g,this.renderMultiDrawInstances=y}function EM(s){const e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function r(u,c,f){switch(t.calls++,c){case s.TRIANGLES:t.triangles+=f*(u/3);break;case s.LINES:t.lines+=f*(u/2);break;case s.LINE_STRIP:t.lines+=f*(u-1);break;case s.LINE_LOOP:t.lines+=f*u;break;case s.POINTS:t.points+=f*u;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",c);break}}function o(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:o,update:r}}function wM(s,e,t){const r=new WeakMap,o=new Zt;function u(c,f,h){const m=c.morphTargetInfluences,g=f.morphAttributes.position||f.morphAttributes.normal||f.morphAttributes.color,y=g!==void 0?g.length:0;let v=r.get(f);if(v===void 0||v.count!==y){let w=function(){H.dispose(),r.delete(f),f.removeEventListener("dispose",w)};var M=w;v!==void 0&&v.texture.dispose();const T=f.morphAttributes.position!==void 0,S=f.morphAttributes.normal!==void 0,x=f.morphAttributes.color!==void 0,_=f.morphAttributes.position||[],P=f.morphAttributes.normal||[],R=f.morphAttributes.color||[];let L=0;T===!0&&(L=1),S===!0&&(L=2),x===!0&&(L=3);let Y=f.attributes.position.count*L,U=1;Y>e.maxTextureSize&&(U=Math.ceil(Y/e.maxTextureSize),Y=e.maxTextureSize);const N=new Float32Array(Y*U*4*y),H=new Kg(N,Y,U,y);H.type=zi,H.needsUpdate=!0;const b=L*4;for(let I=0;I<y;I++){const K=_[I],V=P[I],Z=R[I],te=Y*U*4*I;for(let ue=0;ue<K.count;ue++){const se=ue*b;T===!0&&(o.fromBufferAttribute(K,ue),N[te+se+0]=o.x,N[te+se+1]=o.y,N[te+se+2]=o.z,N[te+se+3]=0),S===!0&&(o.fromBufferAttribute(V,ue),N[te+se+4]=o.x,N[te+se+5]=o.y,N[te+se+6]=o.z,N[te+se+7]=0),x===!0&&(o.fromBufferAttribute(Z,ue),N[te+se+8]=o.x,N[te+se+9]=o.y,N[te+se+10]=o.z,N[te+se+11]=Z.itemSize===4?o.w:1)}}v={count:y,texture:H,size:new ht(Y,U)},r.set(f,v),f.addEventListener("dispose",w)}if(c.isInstancedMesh===!0&&c.morphTexture!==null)h.getUniforms().setValue(s,"morphTexture",c.morphTexture,t);else{let T=0;for(let x=0;x<m.length;x++)T+=m[x];const S=f.morphTargetsRelative?1:1-T;h.getUniforms().setValue(s,"morphTargetBaseInfluence",S),h.getUniforms().setValue(s,"morphTargetInfluences",m)}h.getUniforms().setValue(s,"morphTargetsTexture",v.texture,t),h.getUniforms().setValue(s,"morphTargetsTextureSize",v.size)}return{update:u}}function TM(s,e,t,r){let o=new WeakMap;function u(h){const m=r.render.frame,g=h.geometry,y=e.get(h,g);if(o.get(y)!==m&&(e.update(y),o.set(y,m)),h.isInstancedMesh&&(h.hasEventListener("dispose",f)===!1&&h.addEventListener("dispose",f),o.get(h)!==m&&(t.update(h.instanceMatrix,s.ARRAY_BUFFER),h.instanceColor!==null&&t.update(h.instanceColor,s.ARRAY_BUFFER),o.set(h,m))),h.isSkinnedMesh){const v=h.skeleton;o.get(v)!==m&&(v.update(),o.set(v,m))}return y}function c(){o=new WeakMap}function f(h){const m=h.target;m.removeEventListener("dispose",f),t.remove(m.instanceMatrix),m.instanceColor!==null&&t.remove(m.instanceColor)}return{update:u,dispose:c}}class a_ extends Mn{constructor(e,t,r,o,u,c,f,h,m,g=ea){if(g!==ea&&g!==la)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");r===void 0&&g===ea&&(r=es),r===void 0&&g===la&&(r=oa),super(null,o,u,c,f,h,g,r,m),this.isDepthTexture=!0,this.image={width:e,height:t},this.magFilter=f!==void 0?f:Kn,this.minFilter=h!==void 0?h:Kn,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.compareFunction=e.compareFunction,this}toJSON(e){const t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}}const o_=new Mn,Km=new a_(1,1),l_=new Kg,u_=new fx,c_=new r_,Zm=[],Qm=[],Jm=new Float32Array(16),eg=new Float32Array(9),tg=new Float32Array(4);function fa(s,e,t){const r=s[0];if(r<=0||r>0)return s;const o=e*t;let u=Zm[o];if(u===void 0&&(u=new Float32Array(o),Zm[o]=u),e!==0){r.toArray(u,0);for(let c=1,f=0;c!==e;++c)f+=t,s[c].toArray(u,f)}return u}function Jt(s,e){if(s.length!==e.length)return!1;for(let t=0,r=s.length;t<r;t++)if(s[t]!==e[t])return!1;return!0}function en(s,e){for(let t=0,r=e.length;t<r;t++)s[t]=e[t]}function pu(s,e){let t=Qm[e];t===void 0&&(t=new Int32Array(e),Qm[e]=t);for(let r=0;r!==e;++r)t[r]=s.allocateTextureUnit();return t}function AM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1f(this.addr,e),t[0]=e)}function CM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2fv(this.addr,e),en(t,e)}}function RM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(s.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(Jt(t,e))return;s.uniform3fv(this.addr,e),en(t,e)}}function bM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4fv(this.addr,e),en(t,e)}}function PM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix2fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;tg.set(r),s.uniformMatrix2fv(this.addr,!1,tg),en(t,r)}}function LM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix3fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;eg.set(r),s.uniformMatrix3fv(this.addr,!1,eg),en(t,r)}}function NM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix4fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;Jm.set(r),s.uniformMatrix4fv(this.addr,!1,Jm),en(t,r)}}function DM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1i(this.addr,e),t[0]=e)}function IM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2iv(this.addr,e),en(t,e)}}function UM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Jt(t,e))return;s.uniform3iv(this.addr,e),en(t,e)}}function FM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4iv(this.addr,e),en(t,e)}}function OM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1ui(this.addr,e),t[0]=e)}function kM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2uiv(this.addr,e),en(t,e)}}function BM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Jt(t,e))return;s.uniform3uiv(this.addr,e),en(t,e)}}function zM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4uiv(this.addr,e),en(t,e)}}function HM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o);let u;this.type===s.SAMPLER_2D_SHADOW?(Km.compareFunction=Yg,u=Km):u=o_,t.setTexture2D(e||u,o)}function VM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTexture3D(e||u_,o)}function GM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTextureCube(e||c_,o)}function WM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTexture2DArray(e||l_,o)}function jM(s){switch(s){case 5126:return AM;case 35664:return CM;case 35665:return RM;case 35666:return bM;case 35674:return PM;case 35675:return LM;case 35676:return NM;case 5124:case 35670:return DM;case 35667:case 35671:return IM;case 35668:case 35672:return UM;case 35669:case 35673:return FM;case 5125:return OM;case 36294:return kM;case 36295:return BM;case 36296:return zM;case 35678:case 36198:case 36298:case 36306:case 35682:return HM;case 35679:case 36299:case 36307:return VM;case 35680:case 36300:case 36308:case 36293:return GM;case 36289:case 36303:case 36311:case 36292:return WM}}function XM(s,e){s.uniform1fv(this.addr,e)}function YM(s,e){const t=fa(e,this.size,2);s.uniform2fv(this.addr,t)}function qM(s,e){const t=fa(e,this.size,3);s.uniform3fv(this.addr,t)}function $M(s,e){const t=fa(e,this.size,4);s.uniform4fv(this.addr,t)}function KM(s,e){const t=fa(e,this.size,4);s.uniformMatrix2fv(this.addr,!1,t)}function ZM(s,e){const t=fa(e,this.size,9);s.uniformMatrix3fv(this.addr,!1,t)}function QM(s,e){const t=fa(e,this.size,16);s.uniformMatrix4fv(this.addr,!1,t)}function JM(s,e){s.uniform1iv(this.addr,e)}function eE(s,e){s.uniform2iv(this.addr,e)}function tE(s,e){s.uniform3iv(this.addr,e)}function nE(s,e){s.uniform4iv(this.addr,e)}function iE(s,e){s.uniform1uiv(this.addr,e)}function rE(s,e){s.uniform2uiv(this.addr,e)}function sE(s,e){s.uniform3uiv(this.addr,e)}function aE(s,e){s.uniform4uiv(this.addr,e)}function oE(s,e,t){const r=this.cache,o=e.length,u=pu(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture2D(e[c]||o_,u[c])}function lE(s,e,t){const r=this.cache,o=e.length,u=pu(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture3D(e[c]||u_,u[c])}function uE(s,e,t){const r=this.cache,o=e.length,u=pu(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTextureCube(e[c]||c_,u[c])}function cE(s,e,t){const r=this.cache,o=e.length,u=pu(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture2DArray(e[c]||l_,u[c])}function fE(s){switch(s){case 5126:return XM;case 35664:return YM;case 35665:return qM;case 35666:return $M;case 35674:return KM;case 35675:return ZM;case 35676:return QM;case 5124:case 35670:return JM;case 35667:case 35671:return eE;case 35668:case 35672:return tE;case 35669:case 35673:return nE;case 5125:return iE;case 36294:return rE;case 36295:return sE;case 36296:return aE;case 35678:case 36198:case 36298:case 36306:case 35682:return oE;case 35679:case 36299:case 36307:return lE;case 35680:case 36300:case 36308:case 36293:return uE;case 36289:case 36303:case 36311:case 36292:return cE}}class dE{constructor(e,t,r){this.id=e,this.addr=r,this.cache=[],this.type=t.type,this.setValue=jM(t.type)}}class hE{constructor(e,t,r){this.id=e,this.addr=r,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=fE(t.type)}}class pE{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,r){const o=this.seq;for(let u=0,c=o.length;u!==c;++u){const f=o[u];f.setValue(e,t[f.id],r)}}}const Cf=/(\w+)(\])?(\[|\.)?/g;function ng(s,e){s.seq.push(e),s.map[e.id]=e}function mE(s,e,t){const r=s.name,o=r.length;for(Cf.lastIndex=0;;){const u=Cf.exec(r),c=Cf.lastIndex;let f=u[1];const h=u[2]==="]",m=u[3];if(h&&(f=f|0),m===void 0||m==="["&&c+2===o){ng(t,m===void 0?new dE(f,s,e):new hE(f,s,e));break}else{let y=t.map[f];y===void 0&&(y=new pE(f),ng(t,y)),t=y}}}class Jl{constructor(e,t){this.seq=[],this.map={};const r=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let o=0;o<r;++o){const u=e.getActiveUniform(t,o),c=e.getUniformLocation(t,u.name);mE(u,c,this)}}setValue(e,t,r,o){const u=this.map[t];u!==void 0&&u.setValue(e,r,o)}setOptional(e,t,r){const o=t[r];o!==void 0&&this.setValue(e,r,o)}static upload(e,t,r,o){for(let u=0,c=t.length;u!==c;++u){const f=t[u],h=r[f.id];h.needsUpdate!==!1&&f.setValue(e,h.value,o)}}static seqWithValue(e,t){const r=[];for(let o=0,u=e.length;o!==u;++o){const c=e[o];c.id in t&&r.push(c)}return r}}function ig(s,e,t){const r=s.createShader(e);return s.shaderSource(r,t),s.compileShader(r),r}const gE=37297;let _E=0;function vE(s,e){const t=s.split(`
`),r=[],o=Math.max(e-6,0),u=Math.min(e+6,t.length);for(let c=o;c<u;c++){const f=c+1;r.push(`${f===e?">":" "} ${f}: ${t[c]}`)}return r.join(`
`)}function xE(s){const e=Rt.getPrimaries(Rt.workingColorSpace),t=Rt.getPrimaries(s);let r;switch(e===t?r="":e===ru&&t===iu?r="LinearDisplayP3ToLinearSRGB":e===iu&&t===ru&&(r="LinearSRGBToLinearDisplayP3"),s){case wr:case fu:return[r,"LinearTransferOETF"];case ui:case bd:return[r,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",s),[r,"LinearTransferOETF"]}}function rg(s,e,t){const r=s.getShaderParameter(e,s.COMPILE_STATUS),o=s.getShaderInfoLog(e).trim();if(r&&o==="")return"";const u=/ERROR: 0:(\d+)/.exec(o);if(u){const c=parseInt(u[1]);return t.toUpperCase()+`

`+o+`

`+vE(s.getShaderSource(e),c)}else return o}function yE(s,e){const t=xE(e);return`vec4 ${s}( vec4 value ) { return ${t[0]}( ${t[1]}( value ) ); }`}function SE(s,e){let t;switch(e){case B0:t="Linear";break;case z0:t="Reinhard";break;case H0:t="OptimizedCineon";break;case V0:t="ACESFilmic";break;case W0:t="AgX";break;case j0:t="Neutral";break;case G0:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+s+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}const Ul=new Q;function ME(){Rt.getLuminanceCoefficients(Ul);const s=Ul.x.toFixed(4),e=Ul.y.toFixed(4),t=Ul.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${s}, ${e}, ${t} );`,"	return dot( weights, rgb );","}"].join(`
`)}function EE(s){return[s.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",s.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(to).join(`
`)}function wE(s){const e=[];for(const t in s){const r=s[t];r!==!1&&e.push("#define "+t+" "+r)}return e.join(`
`)}function TE(s,e){const t={},r=s.getProgramParameter(e,s.ACTIVE_ATTRIBUTES);for(let o=0;o<r;o++){const u=s.getActiveAttrib(e,o),c=u.name;let f=1;u.type===s.FLOAT_MAT2&&(f=2),u.type===s.FLOAT_MAT3&&(f=3),u.type===s.FLOAT_MAT4&&(f=4),t[c]={type:u.type,location:s.getAttribLocation(e,c),locationSize:f}}return t}function to(s){return s!==""}function sg(s,e){const t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return s.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function ag(s,e){return s.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}const AE=/^[ \t]*#include +<([\w\d./]+)>/gm;function yd(s){return s.replace(AE,RE)}const CE=new Map;function RE(s,e){let t=pt[e];if(t===void 0){const r=CE.get(e);if(r!==void 0)t=pt[r],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,r);else throw new Error("Can not resolve #include <"+e+">")}return yd(t)}const bE=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function og(s){return s.replace(bE,PE)}function PE(s,e,t,r){let o="";for(let u=parseInt(e);u<parseInt(t);u++)o+=r.replace(/\[\s*i\s*\]/g,"[ "+u+" ]").replace(/UNROLLED_LOOP_INDEX/g,u);return o}function lg(s){let e=`precision ${s.precision} float;
	precision ${s.precision} int;
	precision ${s.precision} sampler2D;
	precision ${s.precision} samplerCube;
	precision ${s.precision} sampler3D;
	precision ${s.precision} sampler2DArray;
	precision ${s.precision} sampler2DShadow;
	precision ${s.precision} samplerCubeShadow;
	precision ${s.precision} sampler2DArrayShadow;
	precision ${s.precision} isampler2D;
	precision ${s.precision} isampler3D;
	precision ${s.precision} isamplerCube;
	precision ${s.precision} isampler2DArray;
	precision ${s.precision} usampler2D;
	precision ${s.precision} usampler3D;
	precision ${s.precision} usamplerCube;
	precision ${s.precision} usampler2DArray;
	`;return s.precision==="highp"?e+=`
#define HIGH_PRECISION`:s.precision==="mediump"?e+=`
#define MEDIUM_PRECISION`:s.precision==="lowp"&&(e+=`
#define LOW_PRECISION`),e}function LE(s){let e="SHADOWMAP_TYPE_BASIC";return s.shadowMapType===Dg?e="SHADOWMAP_TYPE_PCF":s.shadowMapType===f0?e="SHADOWMAP_TYPE_PCF_SOFT":s.shadowMapType===ki&&(e="SHADOWMAP_TYPE_VSM"),e}function NE(s){let e="ENVMAP_TYPE_CUBE";if(s.envMap)switch(s.envMapMode){case sa:case aa:e="ENVMAP_TYPE_CUBE";break;case cu:e="ENVMAP_TYPE_CUBE_UV";break}return e}function DE(s){let e="ENVMAP_MODE_REFLECTION";if(s.envMap)switch(s.envMapMode){case aa:e="ENVMAP_MODE_REFRACTION";break}return e}function IE(s){let e="ENVMAP_BLENDING_NONE";if(s.envMap)switch(s.combine){case Ig:e="ENVMAP_BLENDING_MULTIPLY";break;case O0:e="ENVMAP_BLENDING_MIX";break;case k0:e="ENVMAP_BLENDING_ADD";break}return e}function UE(s){const e=s.envMapCubeUVHeight;if(e===null)return null;const t=Math.log2(e)-2,r=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),112)),texelHeight:r,maxMip:t}}function FE(s,e,t,r){const o=s.getContext(),u=t.defines;let c=t.vertexShader,f=t.fragmentShader;const h=LE(t),m=NE(t),g=DE(t),y=IE(t),v=UE(t),M=EE(t),T=wE(u),S=o.createProgram();let x,_,P=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(x=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T].filter(to).join(`
`),x.length>0&&(x+=`
`),_=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T].filter(to).join(`
`),_.length>0&&(_+=`
`)):(x=[lg(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+g:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+h:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(to).join(`
`),_=[lg(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+m:"",t.envMap?"#define "+g:"",t.envMap?"#define "+y:"",v?"#define CUBEUV_TEXEL_WIDTH "+v.texelWidth:"",v?"#define CUBEUV_TEXEL_HEIGHT "+v.texelHeight:"",v?"#define CUBEUV_MAX_MIP "+v.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+h:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==yr?"#define TONE_MAPPING":"",t.toneMapping!==yr?pt.tonemapping_pars_fragment:"",t.toneMapping!==yr?SE("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",pt.colorspace_pars_fragment,yE("linearToOutputTexel",t.outputColorSpace),ME(),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(to).join(`
`)),c=yd(c),c=sg(c,t),c=ag(c,t),f=yd(f),f=sg(f,t),f=ag(f,t),c=og(c),f=og(f),t.isRawShaderMaterial!==!0&&(P=`#version 300 es
`,x=[M,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+x,_=["#define varying in",t.glslVersion===Em?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===Em?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+_);const R=P+x+c,L=P+_+f,Y=ig(o,o.VERTEX_SHADER,R),U=ig(o,o.FRAGMENT_SHADER,L);o.attachShader(S,Y),o.attachShader(S,U),t.index0AttributeName!==void 0?o.bindAttribLocation(S,0,t.index0AttributeName):t.morphTargets===!0&&o.bindAttribLocation(S,0,"position"),o.linkProgram(S);function N(I){if(s.debug.checkShaderErrors){const K=o.getProgramInfoLog(S).trim(),V=o.getShaderInfoLog(Y).trim(),Z=o.getShaderInfoLog(U).trim();let te=!0,ue=!0;if(o.getProgramParameter(S,o.LINK_STATUS)===!1)if(te=!1,typeof s.debug.onShaderError=="function")s.debug.onShaderError(o,S,Y,U);else{const se=rg(o,Y,"vertex"),G=rg(o,U,"fragment");console.error("THREE.WebGLProgram: Shader Error "+o.getError()+" - VALIDATE_STATUS "+o.getProgramParameter(S,o.VALIDATE_STATUS)+`

Material Name: `+I.name+`
Material Type: `+I.type+`

Program Info Log: `+K+`
`+se+`
`+G)}else K!==""?console.warn("THREE.WebGLProgram: Program Info Log:",K):(V===""||Z==="")&&(ue=!1);ue&&(I.diagnostics={runnable:te,programLog:K,vertexShader:{log:V,prefix:x},fragmentShader:{log:Z,prefix:_}})}o.deleteShader(Y),o.deleteShader(U),H=new Jl(o,S),b=TE(o,S)}let H;this.getUniforms=function(){return H===void 0&&N(this),H};let b;this.getAttributes=function(){return b===void 0&&N(this),b};let w=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return w===!1&&(w=o.getProgramParameter(S,gE)),w},this.destroy=function(){r.releaseStatesOfProgram(this),o.deleteProgram(S),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=_E++,this.cacheKey=e,this.usedTimes=1,this.program=S,this.vertexShader=Y,this.fragmentShader=U,this}let OE=0;class kE{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){const t=e.vertexShader,r=e.fragmentShader,o=this._getShaderStage(t),u=this._getShaderStage(r),c=this._getShaderCacheForMaterial(e);return c.has(o)===!1&&(c.add(o),o.usedTimes++),c.has(u)===!1&&(c.add(u),u.usedTimes++),this}remove(e){const t=this.materialCache.get(e);for(const r of t)r.usedTimes--,r.usedTimes===0&&this.shaderCache.delete(r.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){const t=this.materialCache;let r=t.get(e);return r===void 0&&(r=new Set,t.set(e,r)),r}_getShaderStage(e){const t=this.shaderCache;let r=t.get(e);return r===void 0&&(r=new BE(e),t.set(e,r)),r}}class BE{constructor(e){this.id=OE++,this.code=e,this.usedTimes=0}}function zE(s,e,t,r,o,u,c){const f=new Qg,h=new kE,m=new Set,g=[],y=o.logarithmicDepthBuffer,v=o.vertexTextures;let M=o.precision;const T={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function S(b){return m.add(b),b===0?"uv":`uv${b}`}function x(b,w,I,K,V){const Z=K.fog,te=V.geometry,ue=b.isMeshStandardMaterial?K.environment:null,se=(b.isMeshStandardMaterial?t:e).get(b.envMap||ue),G=se&&se.mapping===cu?se.image.height:null,he=T[b.type];b.precision!==null&&(M=o.getMaxPrecision(b.precision),M!==b.precision&&console.warn("THREE.WebGLProgram.getParameters:",b.precision,"not supported, using",M,"instead."));const fe=te.morphAttributes.position||te.morphAttributes.normal||te.morphAttributes.color,k=fe!==void 0?fe.length:0;let ce=0;te.morphAttributes.position!==void 0&&(ce=1),te.morphAttributes.normal!==void 0&&(ce=2),te.morphAttributes.color!==void 0&&(ce=3);let ke,J,de,Se;if(he){const tt=vi[he];ke=tt.vertexShader,J=tt.fragmentShader}else ke=b.vertexShader,J=b.fragmentShader,h.update(b),de=h.getVertexShaderID(b),Se=h.getFragmentShaderID(b);const Me=s.getRenderTarget(),Ie=V.isInstancedMesh===!0,Ge=V.isBatchedMesh===!0,Qe=!!b.map,Et=!!b.matcap,B=!!se,wt=!!b.aoMap,gt=!!b.lightMap,ne=!!b.bumpMap,ie=!!b.normalMap,De=!!b.displacementMap,Ae=!!b.emissiveMap,Ue=!!b.metalnessMap,F=!!b.roughnessMap,A=b.anisotropy>0,oe=b.clearcoat>0,ve=b.dispersion>0,Ee=b.iridescence>0,ge=b.sheen>0,Ze=b.transmission>0,Ne=A&&!!b.anisotropyMap,Be=oe&&!!b.clearcoatMap,ct=oe&&!!b.clearcoatNormalMap,Te=oe&&!!b.clearcoatRoughnessMap,Oe=Ee&&!!b.iridescenceMap,_t=Ee&&!!b.iridescenceThicknessMap,rt=ge&&!!b.sheenColorMap,Ve=ge&&!!b.sheenRoughnessMap,lt=!!b.specularMap,ft=!!b.specularColorMap,Ct=!!b.specularIntensityMap,j=Ze&&!!b.transmissionMap,Ce=Ze&&!!b.thicknessMap,pe=!!b.gradientMap,me=!!b.alphaMap,Re=b.alphaTest>0,Je=!!b.alphaHash,vt=!!b.extensions;let xe=yr;b.toneMapped&&(Me===null||Me.isXRRenderTarget===!0)&&(xe=s.toneMapping);const at={shaderID:he,shaderType:b.type,shaderName:b.name,vertexShader:ke,fragmentShader:J,defines:b.defines,customVertexShaderID:de,customFragmentShaderID:Se,isRawShaderMaterial:b.isRawShaderMaterial===!0,glslVersion:b.glslVersion,precision:M,batching:Ge,batchingColor:Ge&&V._colorsTexture!==null,instancing:Ie,instancingColor:Ie&&V.instanceColor!==null,instancingMorph:Ie&&V.morphTexture!==null,supportsVertexTextures:v,outputColorSpace:Me===null?s.outputColorSpace:Me.isXRRenderTarget===!0?Me.texture.colorSpace:wr,alphaToCoverage:!!b.alphaToCoverage,map:Qe,matcap:Et,envMap:B,envMapMode:B&&se.mapping,envMapCubeUVHeight:G,aoMap:wt,lightMap:gt,bumpMap:ne,normalMap:ie,displacementMap:v&&De,emissiveMap:Ae,normalMapObjectSpace:ie&&b.normalMapType===$0,normalMapTangentSpace:ie&&b.normalMapType===Xg,metalnessMap:Ue,roughnessMap:F,anisotropy:A,anisotropyMap:Ne,clearcoat:oe,clearcoatMap:Be,clearcoatNormalMap:ct,clearcoatRoughnessMap:Te,dispersion:ve,iridescence:Ee,iridescenceMap:Oe,iridescenceThicknessMap:_t,sheen:ge,sheenColorMap:rt,sheenRoughnessMap:Ve,specularMap:lt,specularColorMap:ft,specularIntensityMap:Ct,transmission:Ze,transmissionMap:j,thicknessMap:Ce,gradientMap:pe,opaque:b.transparent===!1&&b.blending===Js&&b.alphaToCoverage===!1,alphaMap:me,alphaTest:Re,alphaHash:Je,combine:b.combine,mapUv:Qe&&S(b.map.channel),aoMapUv:wt&&S(b.aoMap.channel),lightMapUv:gt&&S(b.lightMap.channel),bumpMapUv:ne&&S(b.bumpMap.channel),normalMapUv:ie&&S(b.normalMap.channel),displacementMapUv:De&&S(b.displacementMap.channel),emissiveMapUv:Ae&&S(b.emissiveMap.channel),metalnessMapUv:Ue&&S(b.metalnessMap.channel),roughnessMapUv:F&&S(b.roughnessMap.channel),anisotropyMapUv:Ne&&S(b.anisotropyMap.channel),clearcoatMapUv:Be&&S(b.clearcoatMap.channel),clearcoatNormalMapUv:ct&&S(b.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Te&&S(b.clearcoatRoughnessMap.channel),iridescenceMapUv:Oe&&S(b.iridescenceMap.channel),iridescenceThicknessMapUv:_t&&S(b.iridescenceThicknessMap.channel),sheenColorMapUv:rt&&S(b.sheenColorMap.channel),sheenRoughnessMapUv:Ve&&S(b.sheenRoughnessMap.channel),specularMapUv:lt&&S(b.specularMap.channel),specularColorMapUv:ft&&S(b.specularColorMap.channel),specularIntensityMapUv:Ct&&S(b.specularIntensityMap.channel),transmissionMapUv:j&&S(b.transmissionMap.channel),thicknessMapUv:Ce&&S(b.thicknessMap.channel),alphaMapUv:me&&S(b.alphaMap.channel),vertexTangents:!!te.attributes.tangent&&(ie||A),vertexColors:b.vertexColors,vertexAlphas:b.vertexColors===!0&&!!te.attributes.color&&te.attributes.color.itemSize===4,pointsUvs:V.isPoints===!0&&!!te.attributes.uv&&(Qe||me),fog:!!Z,useFog:b.fog===!0,fogExp2:!!Z&&Z.isFogExp2,flatShading:b.flatShading===!0,sizeAttenuation:b.sizeAttenuation===!0,logarithmicDepthBuffer:y,skinning:V.isSkinnedMesh===!0,morphTargets:te.morphAttributes.position!==void 0,morphNormals:te.morphAttributes.normal!==void 0,morphColors:te.morphAttributes.color!==void 0,morphTargetsCount:k,morphTextureStride:ce,numDirLights:w.directional.length,numPointLights:w.point.length,numSpotLights:w.spot.length,numSpotLightMaps:w.spotLightMap.length,numRectAreaLights:w.rectArea.length,numHemiLights:w.hemi.length,numDirLightShadows:w.directionalShadowMap.length,numPointLightShadows:w.pointShadowMap.length,numSpotLightShadows:w.spotShadowMap.length,numSpotLightShadowsWithMaps:w.numSpotLightShadowsWithMaps,numLightProbes:w.numLightProbes,numClippingPlanes:c.numPlanes,numClipIntersection:c.numIntersection,dithering:b.dithering,shadowMapEnabled:s.shadowMap.enabled&&I.length>0,shadowMapType:s.shadowMap.type,toneMapping:xe,decodeVideoTexture:Qe&&b.map.isVideoTexture===!0&&Rt.getTransfer(b.map.colorSpace)===Ot,premultipliedAlpha:b.premultipliedAlpha,doubleSided:b.side===Bi,flipSided:b.side===Ln,useDepthPacking:b.depthPacking>=0,depthPacking:b.depthPacking||0,index0AttributeName:b.index0AttributeName,extensionClipCullDistance:vt&&b.extensions.clipCullDistance===!0&&r.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(vt&&b.extensions.multiDraw===!0||Ge)&&r.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:r.has("KHR_parallel_shader_compile"),customProgramCacheKey:b.customProgramCacheKey()};return at.vertexUv1s=m.has(1),at.vertexUv2s=m.has(2),at.vertexUv3s=m.has(3),m.clear(),at}function _(b){const w=[];if(b.shaderID?w.push(b.shaderID):(w.push(b.customVertexShaderID),w.push(b.customFragmentShaderID)),b.defines!==void 0)for(const I in b.defines)w.push(I),w.push(b.defines[I]);return b.isRawShaderMaterial===!1&&(P(w,b),R(w,b),w.push(s.outputColorSpace)),w.push(b.customProgramCacheKey),w.join()}function P(b,w){b.push(w.precision),b.push(w.outputColorSpace),b.push(w.envMapMode),b.push(w.envMapCubeUVHeight),b.push(w.mapUv),b.push(w.alphaMapUv),b.push(w.lightMapUv),b.push(w.aoMapUv),b.push(w.bumpMapUv),b.push(w.normalMapUv),b.push(w.displacementMapUv),b.push(w.emissiveMapUv),b.push(w.metalnessMapUv),b.push(w.roughnessMapUv),b.push(w.anisotropyMapUv),b.push(w.clearcoatMapUv),b.push(w.clearcoatNormalMapUv),b.push(w.clearcoatRoughnessMapUv),b.push(w.iridescenceMapUv),b.push(w.iridescenceThicknessMapUv),b.push(w.sheenColorMapUv),b.push(w.sheenRoughnessMapUv),b.push(w.specularMapUv),b.push(w.specularColorMapUv),b.push(w.specularIntensityMapUv),b.push(w.transmissionMapUv),b.push(w.thicknessMapUv),b.push(w.combine),b.push(w.fogExp2),b.push(w.sizeAttenuation),b.push(w.morphTargetsCount),b.push(w.morphAttributeCount),b.push(w.numDirLights),b.push(w.numPointLights),b.push(w.numSpotLights),b.push(w.numSpotLightMaps),b.push(w.numHemiLights),b.push(w.numRectAreaLights),b.push(w.numDirLightShadows),b.push(w.numPointLightShadows),b.push(w.numSpotLightShadows),b.push(w.numSpotLightShadowsWithMaps),b.push(w.numLightProbes),b.push(w.shadowMapType),b.push(w.toneMapping),b.push(w.numClippingPlanes),b.push(w.numClipIntersection),b.push(w.depthPacking)}function R(b,w){f.disableAll(),w.supportsVertexTextures&&f.enable(0),w.instancing&&f.enable(1),w.instancingColor&&f.enable(2),w.instancingMorph&&f.enable(3),w.matcap&&f.enable(4),w.envMap&&f.enable(5),w.normalMapObjectSpace&&f.enable(6),w.normalMapTangentSpace&&f.enable(7),w.clearcoat&&f.enable(8),w.iridescence&&f.enable(9),w.alphaTest&&f.enable(10),w.vertexColors&&f.enable(11),w.vertexAlphas&&f.enable(12),w.vertexUv1s&&f.enable(13),w.vertexUv2s&&f.enable(14),w.vertexUv3s&&f.enable(15),w.vertexTangents&&f.enable(16),w.anisotropy&&f.enable(17),w.alphaHash&&f.enable(18),w.batching&&f.enable(19),w.dispersion&&f.enable(20),w.batchingColor&&f.enable(21),b.push(f.mask),f.disableAll(),w.fog&&f.enable(0),w.useFog&&f.enable(1),w.flatShading&&f.enable(2),w.logarithmicDepthBuffer&&f.enable(3),w.skinning&&f.enable(4),w.morphTargets&&f.enable(5),w.morphNormals&&f.enable(6),w.morphColors&&f.enable(7),w.premultipliedAlpha&&f.enable(8),w.shadowMapEnabled&&f.enable(9),w.doubleSided&&f.enable(10),w.flipSided&&f.enable(11),w.useDepthPacking&&f.enable(12),w.dithering&&f.enable(13),w.transmission&&f.enable(14),w.sheen&&f.enable(15),w.opaque&&f.enable(16),w.pointsUvs&&f.enable(17),w.decodeVideoTexture&&f.enable(18),w.alphaToCoverage&&f.enable(19),b.push(f.mask)}function L(b){const w=T[b.type];let I;if(w){const K=vi[w];I=Ex.clone(K.uniforms)}else I=b.uniforms;return I}function Y(b,w){let I;for(let K=0,V=g.length;K<V;K++){const Z=g[K];if(Z.cacheKey===w){I=Z,++I.usedTimes;break}}return I===void 0&&(I=new FE(s,w,b,u),g.push(I)),I}function U(b){if(--b.usedTimes===0){const w=g.indexOf(b);g[w]=g[g.length-1],g.pop(),b.destroy()}}function N(b){h.remove(b)}function H(){h.dispose()}return{getParameters:x,getProgramCacheKey:_,getUniforms:L,acquireProgram:Y,releaseProgram:U,releaseShaderCache:N,programs:g,dispose:H}}function HE(){let s=new WeakMap;function e(u){let c=s.get(u);return c===void 0&&(c={},s.set(u,c)),c}function t(u){s.delete(u)}function r(u,c,f){s.get(u)[c]=f}function o(){s=new WeakMap}return{get:e,remove:t,update:r,dispose:o}}function VE(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.material.id!==e.material.id?s.material.id-e.material.id:s.z!==e.z?s.z-e.z:s.id-e.id}function ug(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.z!==e.z?e.z-s.z:s.id-e.id}function cg(){const s=[];let e=0;const t=[],r=[],o=[];function u(){e=0,t.length=0,r.length=0,o.length=0}function c(y,v,M,T,S,x){let _=s[e];return _===void 0?(_={id:y.id,object:y,geometry:v,material:M,groupOrder:T,renderOrder:y.renderOrder,z:S,group:x},s[e]=_):(_.id=y.id,_.object=y,_.geometry=v,_.material=M,_.groupOrder=T,_.renderOrder=y.renderOrder,_.z=S,_.group=x),e++,_}function f(y,v,M,T,S,x){const _=c(y,v,M,T,S,x);M.transmission>0?r.push(_):M.transparent===!0?o.push(_):t.push(_)}function h(y,v,M,T,S,x){const _=c(y,v,M,T,S,x);M.transmission>0?r.unshift(_):M.transparent===!0?o.unshift(_):t.unshift(_)}function m(y,v){t.length>1&&t.sort(y||VE),r.length>1&&r.sort(v||ug),o.length>1&&o.sort(v||ug)}function g(){for(let y=e,v=s.length;y<v;y++){const M=s[y];if(M.id===null)break;M.id=null,M.object=null,M.geometry=null,M.material=null,M.group=null}}return{opaque:t,transmissive:r,transparent:o,init:u,push:f,unshift:h,finish:g,sort:m}}function GE(){let s=new WeakMap;function e(r,o){const u=s.get(r);let c;return u===void 0?(c=new cg,s.set(r,[c])):o>=u.length?(c=new cg,u.push(c)):c=u[o],c}function t(){s=new WeakMap}return{get:e,dispose:t}}function WE(){const s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new Q,color:new yt};break;case"SpotLight":t={position:new Q,direction:new Q,color:new yt,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new Q,color:new yt,distance:0,decay:0};break;case"HemisphereLight":t={direction:new Q,skyColor:new yt,groundColor:new yt};break;case"RectAreaLight":t={color:new yt,position:new Q,halfWidth:new Q,halfHeight:new Q};break}return s[e.id]=t,t}}}function jE(){const s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht};break;case"SpotLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht};break;case"PointLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht,shadowCameraNear:1,shadowCameraFar:1e3};break}return s[e.id]=t,t}}}let XE=0;function YE(s,e){return(e.castShadow?2:0)-(s.castShadow?2:0)+(e.map?1:0)-(s.map?1:0)}function qE(s){const e=new WE,t=jE(),r={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let m=0;m<9;m++)r.probe.push(new Q);const o=new Q,u=new Gt,c=new Gt;function f(m){let g=0,y=0,v=0;for(let b=0;b<9;b++)r.probe[b].set(0,0,0);let M=0,T=0,S=0,x=0,_=0,P=0,R=0,L=0,Y=0,U=0,N=0;m.sort(YE);for(let b=0,w=m.length;b<w;b++){const I=m[b],K=I.color,V=I.intensity,Z=I.distance,te=I.shadow&&I.shadow.map?I.shadow.map.texture:null;if(I.isAmbientLight)g+=K.r*V,y+=K.g*V,v+=K.b*V;else if(I.isLightProbe){for(let ue=0;ue<9;ue++)r.probe[ue].addScaledVector(I.sh.coefficients[ue],V);N++}else if(I.isDirectionalLight){const ue=e.get(I);if(ue.color.copy(I.color).multiplyScalar(I.intensity),I.castShadow){const se=I.shadow,G=t.get(I);G.shadowIntensity=se.intensity,G.shadowBias=se.bias,G.shadowNormalBias=se.normalBias,G.shadowRadius=se.radius,G.shadowMapSize=se.mapSize,r.directionalShadow[M]=G,r.directionalShadowMap[M]=te,r.directionalShadowMatrix[M]=I.shadow.matrix,P++}r.directional[M]=ue,M++}else if(I.isSpotLight){const ue=e.get(I);ue.position.setFromMatrixPosition(I.matrixWorld),ue.color.copy(K).multiplyScalar(V),ue.distance=Z,ue.coneCos=Math.cos(I.angle),ue.penumbraCos=Math.cos(I.angle*(1-I.penumbra)),ue.decay=I.decay,r.spot[S]=ue;const se=I.shadow;if(I.map&&(r.spotLightMap[Y]=I.map,Y++,se.updateMatrices(I),I.castShadow&&U++),r.spotLightMatrix[S]=se.matrix,I.castShadow){const G=t.get(I);G.shadowIntensity=se.intensity,G.shadowBias=se.bias,G.shadowNormalBias=se.normalBias,G.shadowRadius=se.radius,G.shadowMapSize=se.mapSize,r.spotShadow[S]=G,r.spotShadowMap[S]=te,L++}S++}else if(I.isRectAreaLight){const ue=e.get(I);ue.color.copy(K).multiplyScalar(V),ue.halfWidth.set(I.width*.5,0,0),ue.halfHeight.set(0,I.height*.5,0),r.rectArea[x]=ue,x++}else if(I.isPointLight){const ue=e.get(I);if(ue.color.copy(I.color).multiplyScalar(I.intensity),ue.distance=I.distance,ue.decay=I.decay,I.castShadow){const se=I.shadow,G=t.get(I);G.shadowIntensity=se.intensity,G.shadowBias=se.bias,G.shadowNormalBias=se.normalBias,G.shadowRadius=se.radius,G.shadowMapSize=se.mapSize,G.shadowCameraNear=se.camera.near,G.shadowCameraFar=se.camera.far,r.pointShadow[T]=G,r.pointShadowMap[T]=te,r.pointShadowMatrix[T]=I.shadow.matrix,R++}r.point[T]=ue,T++}else if(I.isHemisphereLight){const ue=e.get(I);ue.skyColor.copy(I.color).multiplyScalar(V),ue.groundColor.copy(I.groundColor).multiplyScalar(V),r.hemi[_]=ue,_++}}x>0&&(s.has("OES_texture_float_linear")===!0?(r.rectAreaLTC1=Fe.LTC_FLOAT_1,r.rectAreaLTC2=Fe.LTC_FLOAT_2):(r.rectAreaLTC1=Fe.LTC_HALF_1,r.rectAreaLTC2=Fe.LTC_HALF_2)),r.ambient[0]=g,r.ambient[1]=y,r.ambient[2]=v;const H=r.hash;(H.directionalLength!==M||H.pointLength!==T||H.spotLength!==S||H.rectAreaLength!==x||H.hemiLength!==_||H.numDirectionalShadows!==P||H.numPointShadows!==R||H.numSpotShadows!==L||H.numSpotMaps!==Y||H.numLightProbes!==N)&&(r.directional.length=M,r.spot.length=S,r.rectArea.length=x,r.point.length=T,r.hemi.length=_,r.directionalShadow.length=P,r.directionalShadowMap.length=P,r.pointShadow.length=R,r.pointShadowMap.length=R,r.spotShadow.length=L,r.spotShadowMap.length=L,r.directionalShadowMatrix.length=P,r.pointShadowMatrix.length=R,r.spotLightMatrix.length=L+Y-U,r.spotLightMap.length=Y,r.numSpotLightShadowsWithMaps=U,r.numLightProbes=N,H.directionalLength=M,H.pointLength=T,H.spotLength=S,H.rectAreaLength=x,H.hemiLength=_,H.numDirectionalShadows=P,H.numPointShadows=R,H.numSpotShadows=L,H.numSpotMaps=Y,H.numLightProbes=N,r.version=XE++)}function h(m,g){let y=0,v=0,M=0,T=0,S=0;const x=g.matrixWorldInverse;for(let _=0,P=m.length;_<P;_++){const R=m[_];if(R.isDirectionalLight){const L=r.directional[y];L.direction.setFromMatrixPosition(R.matrixWorld),o.setFromMatrixPosition(R.target.matrixWorld),L.direction.sub(o),L.direction.transformDirection(x),y++}else if(R.isSpotLight){const L=r.spot[M];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),L.direction.setFromMatrixPosition(R.matrixWorld),o.setFromMatrixPosition(R.target.matrixWorld),L.direction.sub(o),L.direction.transformDirection(x),M++}else if(R.isRectAreaLight){const L=r.rectArea[T];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),c.identity(),u.copy(R.matrixWorld),u.premultiply(x),c.extractRotation(u),L.halfWidth.set(R.width*.5,0,0),L.halfHeight.set(0,R.height*.5,0),L.halfWidth.applyMatrix4(c),L.halfHeight.applyMatrix4(c),T++}else if(R.isPointLight){const L=r.point[v];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),v++}else if(R.isHemisphereLight){const L=r.hemi[S];L.direction.setFromMatrixPosition(R.matrixWorld),L.direction.transformDirection(x),S++}}}return{setup:f,setupView:h,state:r}}function fg(s){const e=new qE(s),t=[],r=[];function o(g){m.camera=g,t.length=0,r.length=0}function u(g){t.push(g)}function c(g){r.push(g)}function f(){e.setup(t)}function h(g){e.setupView(t,g)}const m={lightsArray:t,shadowsArray:r,camera:null,lights:e,transmissionRenderTarget:{}};return{init:o,state:m,setupLights:f,setupLightsView:h,pushLight:u,pushShadow:c}}function $E(s){let e=new WeakMap;function t(o,u=0){const c=e.get(o);let f;return c===void 0?(f=new fg(s),e.set(o,[f])):u>=c.length?(f=new fg(s),c.push(f)):f=c[u],f}function r(){e=new WeakMap}return{get:t,dispose:r}}class KE extends is{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=Y0,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}}class ZE extends is{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}}const QE=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,JE=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function ew(s,e,t){let r=new Ld;const o=new ht,u=new ht,c=new Zt,f=new KE({depthPacking:q0}),h=new ZE,m={},g=t.maxTextureSize,y={[Mr]:Ln,[Ln]:Mr,[Bi]:Bi},v=new Er({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new ht},radius:{value:4}},vertexShader:QE,fragmentShader:JE}),M=v.clone();M.defines.HORIZONTAL_PASS=1;const T=new Hn;T.setAttribute("position",new hi(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const S=new zn(T,v),x=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=Dg;let _=this.type;this.render=function(U,N,H){if(x.enabled===!1||x.autoUpdate===!1&&x.needsUpdate===!1||U.length===0)return;const b=s.getRenderTarget(),w=s.getActiveCubeFace(),I=s.getActiveMipmapLevel(),K=s.state;K.setBlending(xr),K.buffers.color.setClear(1,1,1,1),K.buffers.depth.setTest(!0),K.setScissorTest(!1);const V=_!==ki&&this.type===ki,Z=_===ki&&this.type!==ki;for(let te=0,ue=U.length;te<ue;te++){const se=U[te],G=se.shadow;if(G===void 0){console.warn("THREE.WebGLShadowMap:",se,"has no shadow.");continue}if(G.autoUpdate===!1&&G.needsUpdate===!1)continue;o.copy(G.mapSize);const he=G.getFrameExtents();if(o.multiply(he),u.copy(G.mapSize),(o.x>g||o.y>g)&&(o.x>g&&(u.x=Math.floor(g/he.x),o.x=u.x*he.x,G.mapSize.x=u.x),o.y>g&&(u.y=Math.floor(g/he.y),o.y=u.y*he.y,G.mapSize.y=u.y)),G.map===null||V===!0||Z===!0){const k=this.type!==ki?{minFilter:Kn,magFilter:Kn}:{};G.map!==null&&G.map.dispose(),G.map=new ts(o.x,o.y,k),G.map.texture.name=se.name+".shadowMap",G.camera.updateProjectionMatrix()}s.setRenderTarget(G.map),s.clear();const fe=G.getViewportCount();for(let k=0;k<fe;k++){const ce=G.getViewport(k);c.set(u.x*ce.x,u.y*ce.y,u.x*ce.z,u.y*ce.w),K.viewport(c),G.updateMatrices(se,k),r=G.getFrustum(),L(N,H,G.camera,se,this.type)}G.isPointLightShadow!==!0&&this.type===ki&&P(G,H),G.needsUpdate=!1}_=this.type,x.needsUpdate=!1,s.setRenderTarget(b,w,I)};function P(U,N){const H=e.update(S);v.defines.VSM_SAMPLES!==U.blurSamples&&(v.defines.VSM_SAMPLES=U.blurSamples,M.defines.VSM_SAMPLES=U.blurSamples,v.needsUpdate=!0,M.needsUpdate=!0),U.mapPass===null&&(U.mapPass=new ts(o.x,o.y)),v.uniforms.shadow_pass.value=U.map.texture,v.uniforms.resolution.value=U.mapSize,v.uniforms.radius.value=U.radius,s.setRenderTarget(U.mapPass),s.clear(),s.renderBufferDirect(N,null,H,v,S,null),M.uniforms.shadow_pass.value=U.mapPass.texture,M.uniforms.resolution.value=U.mapSize,M.uniforms.radius.value=U.radius,s.setRenderTarget(U.map),s.clear(),s.renderBufferDirect(N,null,H,M,S,null)}function R(U,N,H,b){let w=null;const I=H.isPointLight===!0?U.customDistanceMaterial:U.customDepthMaterial;if(I!==void 0)w=I;else if(w=H.isPointLight===!0?h:f,s.localClippingEnabled&&N.clipShadows===!0&&Array.isArray(N.clippingPlanes)&&N.clippingPlanes.length!==0||N.displacementMap&&N.displacementScale!==0||N.alphaMap&&N.alphaTest>0||N.map&&N.alphaTest>0){const K=w.uuid,V=N.uuid;let Z=m[K];Z===void 0&&(Z={},m[K]=Z);let te=Z[V];te===void 0&&(te=w.clone(),Z[V]=te,N.addEventListener("dispose",Y)),w=te}if(w.visible=N.visible,w.wireframe=N.wireframe,b===ki?w.side=N.shadowSide!==null?N.shadowSide:N.side:w.side=N.shadowSide!==null?N.shadowSide:y[N.side],w.alphaMap=N.alphaMap,w.alphaTest=N.alphaTest,w.map=N.map,w.clipShadows=N.clipShadows,w.clippingPlanes=N.clippingPlanes,w.clipIntersection=N.clipIntersection,w.displacementMap=N.displacementMap,w.displacementScale=N.displacementScale,w.displacementBias=N.displacementBias,w.wireframeLinewidth=N.wireframeLinewidth,w.linewidth=N.linewidth,H.isPointLight===!0&&w.isMeshDistanceMaterial===!0){const K=s.properties.get(w);K.light=H}return w}function L(U,N,H,b,w){if(U.visible===!1)return;if(U.layers.test(N.layers)&&(U.isMesh||U.isLine||U.isPoints)&&(U.castShadow||U.receiveShadow&&w===ki)&&(!U.frustumCulled||r.intersectsObject(U))){U.modelViewMatrix.multiplyMatrices(H.matrixWorldInverse,U.matrixWorld);const V=e.update(U),Z=U.material;if(Array.isArray(Z)){const te=V.groups;for(let ue=0,se=te.length;ue<se;ue++){const G=te[ue],he=Z[G.materialIndex];if(he&&he.visible){const fe=R(U,he,b,w);U.onBeforeShadow(s,U,N,H,V,fe,G),s.renderBufferDirect(H,null,V,fe,U,G),U.onAfterShadow(s,U,N,H,V,fe,G)}}}else if(Z.visible){const te=R(U,Z,b,w);U.onBeforeShadow(s,U,N,H,V,te,null),s.renderBufferDirect(H,null,V,te,U,null),U.onAfterShadow(s,U,N,H,V,te,null)}}const K=U.children;for(let V=0,Z=K.length;V<Z;V++)L(K[V],N,H,b,w)}function Y(U){U.target.removeEventListener("dispose",Y);for(const H in m){const b=m[H],w=U.target.uuid;w in b&&(b[w].dispose(),delete b[w])}}}function tw(s){function e(){let j=!1;const Ce=new Zt;let pe=null;const me=new Zt(0,0,0,0);return{setMask:function(Re){pe!==Re&&!j&&(s.colorMask(Re,Re,Re,Re),pe=Re)},setLocked:function(Re){j=Re},setClear:function(Re,Je,vt,xe,at){at===!0&&(Re*=xe,Je*=xe,vt*=xe),Ce.set(Re,Je,vt,xe),me.equals(Ce)===!1&&(s.clearColor(Re,Je,vt,xe),me.copy(Ce))},reset:function(){j=!1,pe=null,me.set(-1,0,0,0)}}}function t(){let j=!1,Ce=null,pe=null,me=null;return{setTest:function(Re){Re?Se(s.DEPTH_TEST):Me(s.DEPTH_TEST)},setMask:function(Re){Ce!==Re&&!j&&(s.depthMask(Re),Ce=Re)},setFunc:function(Re){if(pe!==Re){switch(Re){case P0:s.depthFunc(s.NEVER);break;case L0:s.depthFunc(s.ALWAYS);break;case N0:s.depthFunc(s.LESS);break;case tu:s.depthFunc(s.LEQUAL);break;case D0:s.depthFunc(s.EQUAL);break;case I0:s.depthFunc(s.GEQUAL);break;case U0:s.depthFunc(s.GREATER);break;case F0:s.depthFunc(s.NOTEQUAL);break;default:s.depthFunc(s.LEQUAL)}pe=Re}},setLocked:function(Re){j=Re},setClear:function(Re){me!==Re&&(s.clearDepth(Re),me=Re)},reset:function(){j=!1,Ce=null,pe=null,me=null}}}function r(){let j=!1,Ce=null,pe=null,me=null,Re=null,Je=null,vt=null,xe=null,at=null;return{setTest:function(tt){j||(tt?Se(s.STENCIL_TEST):Me(s.STENCIL_TEST))},setMask:function(tt){Ce!==tt&&!j&&(s.stencilMask(tt),Ce=tt)},setFunc:function(tt,kt,qt){(pe!==tt||me!==kt||Re!==qt)&&(s.stencilFunc(tt,kt,qt),pe=tt,me=kt,Re=qt)},setOp:function(tt,kt,qt){(Je!==tt||vt!==kt||xe!==qt)&&(s.stencilOp(tt,kt,qt),Je=tt,vt=kt,xe=qt)},setLocked:function(tt){j=tt},setClear:function(tt){at!==tt&&(s.clearStencil(tt),at=tt)},reset:function(){j=!1,Ce=null,pe=null,me=null,Re=null,Je=null,vt=null,xe=null,at=null}}}const o=new e,u=new t,c=new r,f=new WeakMap,h=new WeakMap;let m={},g={},y=new WeakMap,v=[],M=null,T=!1,S=null,x=null,_=null,P=null,R=null,L=null,Y=null,U=new yt(0,0,0),N=0,H=!1,b=null,w=null,I=null,K=null,V=null;const Z=s.getParameter(s.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let te=!1,ue=0;const se=s.getParameter(s.VERSION);se.indexOf("WebGL")!==-1?(ue=parseFloat(/^WebGL (\d)/.exec(se)[1]),te=ue>=1):se.indexOf("OpenGL ES")!==-1&&(ue=parseFloat(/^OpenGL ES (\d)/.exec(se)[1]),te=ue>=2);let G=null,he={};const fe=s.getParameter(s.SCISSOR_BOX),k=s.getParameter(s.VIEWPORT),ce=new Zt().fromArray(fe),ke=new Zt().fromArray(k);function J(j,Ce,pe,me){const Re=new Uint8Array(4),Je=s.createTexture();s.bindTexture(j,Je),s.texParameteri(j,s.TEXTURE_MIN_FILTER,s.NEAREST),s.texParameteri(j,s.TEXTURE_MAG_FILTER,s.NEAREST);for(let vt=0;vt<pe;vt++)j===s.TEXTURE_3D||j===s.TEXTURE_2D_ARRAY?s.texImage3D(Ce,0,s.RGBA,1,1,me,0,s.RGBA,s.UNSIGNED_BYTE,Re):s.texImage2D(Ce+vt,0,s.RGBA,1,1,0,s.RGBA,s.UNSIGNED_BYTE,Re);return Je}const de={};de[s.TEXTURE_2D]=J(s.TEXTURE_2D,s.TEXTURE_2D,1),de[s.TEXTURE_CUBE_MAP]=J(s.TEXTURE_CUBE_MAP,s.TEXTURE_CUBE_MAP_POSITIVE_X,6),de[s.TEXTURE_2D_ARRAY]=J(s.TEXTURE_2D_ARRAY,s.TEXTURE_2D_ARRAY,1,1),de[s.TEXTURE_3D]=J(s.TEXTURE_3D,s.TEXTURE_3D,1,1),o.setClear(0,0,0,1),u.setClear(1),c.setClear(0),Se(s.DEPTH_TEST),u.setFunc(tu),ne(!1),ie(vm),Se(s.CULL_FACE),wt(xr);function Se(j){m[j]!==!0&&(s.enable(j),m[j]=!0)}function Me(j){m[j]!==!1&&(s.disable(j),m[j]=!1)}function Ie(j,Ce){return g[j]!==Ce?(s.bindFramebuffer(j,Ce),g[j]=Ce,j===s.DRAW_FRAMEBUFFER&&(g[s.FRAMEBUFFER]=Ce),j===s.FRAMEBUFFER&&(g[s.DRAW_FRAMEBUFFER]=Ce),!0):!1}function Ge(j,Ce){let pe=v,me=!1;if(j){pe=y.get(Ce),pe===void 0&&(pe=[],y.set(Ce,pe));const Re=j.textures;if(pe.length!==Re.length||pe[0]!==s.COLOR_ATTACHMENT0){for(let Je=0,vt=Re.length;Je<vt;Je++)pe[Je]=s.COLOR_ATTACHMENT0+Je;pe.length=Re.length,me=!0}}else pe[0]!==s.BACK&&(pe[0]=s.BACK,me=!0);me&&s.drawBuffers(pe)}function Qe(j){return M!==j?(s.useProgram(j),M=j,!0):!1}const Et={[qr]:s.FUNC_ADD,[h0]:s.FUNC_SUBTRACT,[p0]:s.FUNC_REVERSE_SUBTRACT};Et[m0]=s.MIN,Et[g0]=s.MAX;const B={[_0]:s.ZERO,[v0]:s.ONE,[x0]:s.SRC_COLOR,[zf]:s.SRC_ALPHA,[T0]:s.SRC_ALPHA_SATURATE,[E0]:s.DST_COLOR,[S0]:s.DST_ALPHA,[y0]:s.ONE_MINUS_SRC_COLOR,[Hf]:s.ONE_MINUS_SRC_ALPHA,[w0]:s.ONE_MINUS_DST_COLOR,[M0]:s.ONE_MINUS_DST_ALPHA,[A0]:s.CONSTANT_COLOR,[C0]:s.ONE_MINUS_CONSTANT_COLOR,[R0]:s.CONSTANT_ALPHA,[b0]:s.ONE_MINUS_CONSTANT_ALPHA};function wt(j,Ce,pe,me,Re,Je,vt,xe,at,tt){if(j===xr){T===!0&&(Me(s.BLEND),T=!1);return}if(T===!1&&(Se(s.BLEND),T=!0),j!==d0){if(j!==S||tt!==H){if((x!==qr||R!==qr)&&(s.blendEquation(s.FUNC_ADD),x=qr,R=qr),tt)switch(j){case Js:s.blendFuncSeparate(s.ONE,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case xm:s.blendFunc(s.ONE,s.ONE);break;case ym:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case Sm:s.blendFuncSeparate(s.ZERO,s.SRC_COLOR,s.ZERO,s.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",j);break}else switch(j){case Js:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case xm:s.blendFunc(s.SRC_ALPHA,s.ONE);break;case ym:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case Sm:s.blendFunc(s.ZERO,s.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",j);break}_=null,P=null,L=null,Y=null,U.set(0,0,0),N=0,S=j,H=tt}return}Re=Re||Ce,Je=Je||pe,vt=vt||me,(Ce!==x||Re!==R)&&(s.blendEquationSeparate(Et[Ce],Et[Re]),x=Ce,R=Re),(pe!==_||me!==P||Je!==L||vt!==Y)&&(s.blendFuncSeparate(B[pe],B[me],B[Je],B[vt]),_=pe,P=me,L=Je,Y=vt),(xe.equals(U)===!1||at!==N)&&(s.blendColor(xe.r,xe.g,xe.b,at),U.copy(xe),N=at),S=j,H=!1}function gt(j,Ce){j.side===Bi?Me(s.CULL_FACE):Se(s.CULL_FACE);let pe=j.side===Ln;Ce&&(pe=!pe),ne(pe),j.blending===Js&&j.transparent===!1?wt(xr):wt(j.blending,j.blendEquation,j.blendSrc,j.blendDst,j.blendEquationAlpha,j.blendSrcAlpha,j.blendDstAlpha,j.blendColor,j.blendAlpha,j.premultipliedAlpha),u.setFunc(j.depthFunc),u.setTest(j.depthTest),u.setMask(j.depthWrite),o.setMask(j.colorWrite);const me=j.stencilWrite;c.setTest(me),me&&(c.setMask(j.stencilWriteMask),c.setFunc(j.stencilFunc,j.stencilRef,j.stencilFuncMask),c.setOp(j.stencilFail,j.stencilZFail,j.stencilZPass)),Ae(j.polygonOffset,j.polygonOffsetFactor,j.polygonOffsetUnits),j.alphaToCoverage===!0?Se(s.SAMPLE_ALPHA_TO_COVERAGE):Me(s.SAMPLE_ALPHA_TO_COVERAGE)}function ne(j){b!==j&&(j?s.frontFace(s.CW):s.frontFace(s.CCW),b=j)}function ie(j){j!==u0?(Se(s.CULL_FACE),j!==w&&(j===vm?s.cullFace(s.BACK):j===c0?s.cullFace(s.FRONT):s.cullFace(s.FRONT_AND_BACK))):Me(s.CULL_FACE),w=j}function De(j){j!==I&&(te&&s.lineWidth(j),I=j)}function Ae(j,Ce,pe){j?(Se(s.POLYGON_OFFSET_FILL),(K!==Ce||V!==pe)&&(s.polygonOffset(Ce,pe),K=Ce,V=pe)):Me(s.POLYGON_OFFSET_FILL)}function Ue(j){j?Se(s.SCISSOR_TEST):Me(s.SCISSOR_TEST)}function F(j){j===void 0&&(j=s.TEXTURE0+Z-1),G!==j&&(s.activeTexture(j),G=j)}function A(j,Ce,pe){pe===void 0&&(G===null?pe=s.TEXTURE0+Z-1:pe=G);let me=he[pe];me===void 0&&(me={type:void 0,texture:void 0},he[pe]=me),(me.type!==j||me.texture!==Ce)&&(G!==pe&&(s.activeTexture(pe),G=pe),s.bindTexture(j,Ce||de[j]),me.type=j,me.texture=Ce)}function oe(){const j=he[G];j!==void 0&&j.type!==void 0&&(s.bindTexture(j.type,null),j.type=void 0,j.texture=void 0)}function ve(){try{s.compressedTexImage2D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Ee(){try{s.compressedTexImage3D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function ge(){try{s.texSubImage2D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Ze(){try{s.texSubImage3D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Ne(){try{s.compressedTexSubImage2D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Be(){try{s.compressedTexSubImage3D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function ct(){try{s.texStorage2D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Te(){try{s.texStorage3D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function Oe(){try{s.texImage2D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function _t(){try{s.texImage3D.apply(s,arguments)}catch(j){console.error("THREE.WebGLState:",j)}}function rt(j){ce.equals(j)===!1&&(s.scissor(j.x,j.y,j.z,j.w),ce.copy(j))}function Ve(j){ke.equals(j)===!1&&(s.viewport(j.x,j.y,j.z,j.w),ke.copy(j))}function lt(j,Ce){let pe=h.get(Ce);pe===void 0&&(pe=new WeakMap,h.set(Ce,pe));let me=pe.get(j);me===void 0&&(me=s.getUniformBlockIndex(Ce,j.name),pe.set(j,me))}function ft(j,Ce){const me=h.get(Ce).get(j);f.get(Ce)!==me&&(s.uniformBlockBinding(Ce,me,j.__bindingPointIndex),f.set(Ce,me))}function Ct(){s.disable(s.BLEND),s.disable(s.CULL_FACE),s.disable(s.DEPTH_TEST),s.disable(s.POLYGON_OFFSET_FILL),s.disable(s.SCISSOR_TEST),s.disable(s.STENCIL_TEST),s.disable(s.SAMPLE_ALPHA_TO_COVERAGE),s.blendEquation(s.FUNC_ADD),s.blendFunc(s.ONE,s.ZERO),s.blendFuncSeparate(s.ONE,s.ZERO,s.ONE,s.ZERO),s.blendColor(0,0,0,0),s.colorMask(!0,!0,!0,!0),s.clearColor(0,0,0,0),s.depthMask(!0),s.depthFunc(s.LESS),s.clearDepth(1),s.stencilMask(4294967295),s.stencilFunc(s.ALWAYS,0,4294967295),s.stencilOp(s.KEEP,s.KEEP,s.KEEP),s.clearStencil(0),s.cullFace(s.BACK),s.frontFace(s.CCW),s.polygonOffset(0,0),s.activeTexture(s.TEXTURE0),s.bindFramebuffer(s.FRAMEBUFFER,null),s.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),s.bindFramebuffer(s.READ_FRAMEBUFFER,null),s.useProgram(null),s.lineWidth(1),s.scissor(0,0,s.canvas.width,s.canvas.height),s.viewport(0,0,s.canvas.width,s.canvas.height),m={},G=null,he={},g={},y=new WeakMap,v=[],M=null,T=!1,S=null,x=null,_=null,P=null,R=null,L=null,Y=null,U=new yt(0,0,0),N=0,H=!1,b=null,w=null,I=null,K=null,V=null,ce.set(0,0,s.canvas.width,s.canvas.height),ke.set(0,0,s.canvas.width,s.canvas.height),o.reset(),u.reset(),c.reset()}return{buffers:{color:o,depth:u,stencil:c},enable:Se,disable:Me,bindFramebuffer:Ie,drawBuffers:Ge,useProgram:Qe,setBlending:wt,setMaterial:gt,setFlipSided:ne,setCullFace:ie,setLineWidth:De,setPolygonOffset:Ae,setScissorTest:Ue,activeTexture:F,bindTexture:A,unbindTexture:oe,compressedTexImage2D:ve,compressedTexImage3D:Ee,texImage2D:Oe,texImage3D:_t,updateUBOMapping:lt,uniformBlockBinding:ft,texStorage2D:ct,texStorage3D:Te,texSubImage2D:ge,texSubImage3D:Ze,compressedTexSubImage2D:Ne,compressedTexSubImage3D:Be,scissor:rt,viewport:Ve,reset:Ct}}function dg(s,e,t,r){const o=nw(r);switch(t){case Bg:return s*e;case Hg:return s*e;case Vg:return s*e*2;case Gg:return s*e/o.components*o.byteLength;case Ad:return s*e/o.components*o.byteLength;case Wg:return s*e*2/o.components*o.byteLength;case Cd:return s*e*2/o.components*o.byteLength;case zg:return s*e*3/o.components*o.byteLength;case di:return s*e*4/o.components*o.byteLength;case Rd:return s*e*4/o.components*o.byteLength;case ql:case $l:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case Kl:case Zl:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case Yf:case $f:return Math.max(s,16)*Math.max(e,8)/4;case Xf:case qf:return Math.max(s,8)*Math.max(e,8)/2;case Kf:case Zf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case Qf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case Jf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case ed:return Math.floor((s+4)/5)*Math.floor((e+3)/4)*16;case td:return Math.floor((s+4)/5)*Math.floor((e+4)/5)*16;case nd:return Math.floor((s+5)/6)*Math.floor((e+4)/5)*16;case id:return Math.floor((s+5)/6)*Math.floor((e+5)/6)*16;case rd:return Math.floor((s+7)/8)*Math.floor((e+4)/5)*16;case sd:return Math.floor((s+7)/8)*Math.floor((e+5)/6)*16;case ad:return Math.floor((s+7)/8)*Math.floor((e+7)/8)*16;case od:return Math.floor((s+9)/10)*Math.floor((e+4)/5)*16;case ld:return Math.floor((s+9)/10)*Math.floor((e+5)/6)*16;case ud:return Math.floor((s+9)/10)*Math.floor((e+7)/8)*16;case cd:return Math.floor((s+9)/10)*Math.floor((e+9)/10)*16;case fd:return Math.floor((s+11)/12)*Math.floor((e+9)/10)*16;case dd:return Math.floor((s+11)/12)*Math.floor((e+11)/12)*16;case Ql:case hd:case pd:return Math.ceil(s/4)*Math.ceil(e/4)*16;case jg:case md:return Math.ceil(s/4)*Math.ceil(e/4)*8;case gd:case _d:return Math.ceil(s/4)*Math.ceil(e/4)*16}throw new Error(`Unable to determine texture byte length for ${t} format.`)}function nw(s){switch(s){case Vi:case Fg:return{byteLength:1,components:1};case no:case Og:case ro:return{byteLength:2,components:1};case wd:case Td:return{byteLength:2,components:4};case es:case Ed:case zi:return{byteLength:4,components:1};case kg:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${s}.`)}function iw(s,e,t,r,o,u,c){const f=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,h=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),m=new ht,g=new WeakMap;let y;const v=new WeakMap;let M=!1;try{M=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function T(F,A){return M?new OffscreenCanvas(F,A):au("canvas")}function S(F,A,oe){let ve=1;const Ee=Ue(F);if((Ee.width>oe||Ee.height>oe)&&(ve=oe/Math.max(Ee.width,Ee.height)),ve<1)if(typeof HTMLImageElement<"u"&&F instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&F instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&F instanceof ImageBitmap||typeof VideoFrame<"u"&&F instanceof VideoFrame){const ge=Math.floor(ve*Ee.width),Ze=Math.floor(ve*Ee.height);y===void 0&&(y=T(ge,Ze));const Ne=A?T(ge,Ze):y;return Ne.width=ge,Ne.height=Ze,Ne.getContext("2d").drawImage(F,0,0,ge,Ze),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+Ee.width+"x"+Ee.height+") to ("+ge+"x"+Ze+")."),Ne}else return"data"in F&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+Ee.width+"x"+Ee.height+")."),F;return F}function x(F){return F.generateMipmaps&&F.minFilter!==Kn&&F.minFilter!==ci}function _(F){s.generateMipmap(F)}function P(F,A,oe,ve,Ee=!1){if(F!==null){if(s[F]!==void 0)return s[F];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+F+"'")}let ge=A;if(A===s.RED&&(oe===s.FLOAT&&(ge=s.R32F),oe===s.HALF_FLOAT&&(ge=s.R16F),oe===s.UNSIGNED_BYTE&&(ge=s.R8)),A===s.RED_INTEGER&&(oe===s.UNSIGNED_BYTE&&(ge=s.R8UI),oe===s.UNSIGNED_SHORT&&(ge=s.R16UI),oe===s.UNSIGNED_INT&&(ge=s.R32UI),oe===s.BYTE&&(ge=s.R8I),oe===s.SHORT&&(ge=s.R16I),oe===s.INT&&(ge=s.R32I)),A===s.RG&&(oe===s.FLOAT&&(ge=s.RG32F),oe===s.HALF_FLOAT&&(ge=s.RG16F),oe===s.UNSIGNED_BYTE&&(ge=s.RG8)),A===s.RG_INTEGER&&(oe===s.UNSIGNED_BYTE&&(ge=s.RG8UI),oe===s.UNSIGNED_SHORT&&(ge=s.RG16UI),oe===s.UNSIGNED_INT&&(ge=s.RG32UI),oe===s.BYTE&&(ge=s.RG8I),oe===s.SHORT&&(ge=s.RG16I),oe===s.INT&&(ge=s.RG32I)),A===s.RGB&&oe===s.UNSIGNED_INT_5_9_9_9_REV&&(ge=s.RGB9_E5),A===s.RGBA){const Ze=Ee?nu:Rt.getTransfer(ve);oe===s.FLOAT&&(ge=s.RGBA32F),oe===s.HALF_FLOAT&&(ge=s.RGBA16F),oe===s.UNSIGNED_BYTE&&(ge=Ze===Ot?s.SRGB8_ALPHA8:s.RGBA8),oe===s.UNSIGNED_SHORT_4_4_4_4&&(ge=s.RGBA4),oe===s.UNSIGNED_SHORT_5_5_5_1&&(ge=s.RGB5_A1)}return(ge===s.R16F||ge===s.R32F||ge===s.RG16F||ge===s.RG32F||ge===s.RGBA16F||ge===s.RGBA32F)&&e.get("EXT_color_buffer_float"),ge}function R(F,A){let oe;return F?A===null||A===es||A===oa?oe=s.DEPTH24_STENCIL8:A===zi?oe=s.DEPTH32F_STENCIL8:A===no&&(oe=s.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):A===null||A===es||A===oa?oe=s.DEPTH_COMPONENT24:A===zi?oe=s.DEPTH_COMPONENT32F:A===no&&(oe=s.DEPTH_COMPONENT16),oe}function L(F,A){return x(F)===!0||F.isFramebufferTexture&&F.minFilter!==Kn&&F.minFilter!==ci?Math.log2(Math.max(A.width,A.height))+1:F.mipmaps!==void 0&&F.mipmaps.length>0?F.mipmaps.length:F.isCompressedTexture&&Array.isArray(F.image)?A.mipmaps.length:1}function Y(F){const A=F.target;A.removeEventListener("dispose",Y),N(A),A.isVideoTexture&&g.delete(A)}function U(F){const A=F.target;A.removeEventListener("dispose",U),b(A)}function N(F){const A=r.get(F);if(A.__webglInit===void 0)return;const oe=F.source,ve=v.get(oe);if(ve){const Ee=ve[A.__cacheKey];Ee.usedTimes--,Ee.usedTimes===0&&H(F),Object.keys(ve).length===0&&v.delete(oe)}r.remove(F)}function H(F){const A=r.get(F);s.deleteTexture(A.__webglTexture);const oe=F.source,ve=v.get(oe);delete ve[A.__cacheKey],c.memory.textures--}function b(F){const A=r.get(F);if(F.depthTexture&&F.depthTexture.dispose(),F.isWebGLCubeRenderTarget)for(let ve=0;ve<6;ve++){if(Array.isArray(A.__webglFramebuffer[ve]))for(let Ee=0;Ee<A.__webglFramebuffer[ve].length;Ee++)s.deleteFramebuffer(A.__webglFramebuffer[ve][Ee]);else s.deleteFramebuffer(A.__webglFramebuffer[ve]);A.__webglDepthbuffer&&s.deleteRenderbuffer(A.__webglDepthbuffer[ve])}else{if(Array.isArray(A.__webglFramebuffer))for(let ve=0;ve<A.__webglFramebuffer.length;ve++)s.deleteFramebuffer(A.__webglFramebuffer[ve]);else s.deleteFramebuffer(A.__webglFramebuffer);if(A.__webglDepthbuffer&&s.deleteRenderbuffer(A.__webglDepthbuffer),A.__webglMultisampledFramebuffer&&s.deleteFramebuffer(A.__webglMultisampledFramebuffer),A.__webglColorRenderbuffer)for(let ve=0;ve<A.__webglColorRenderbuffer.length;ve++)A.__webglColorRenderbuffer[ve]&&s.deleteRenderbuffer(A.__webglColorRenderbuffer[ve]);A.__webglDepthRenderbuffer&&s.deleteRenderbuffer(A.__webglDepthRenderbuffer)}const oe=F.textures;for(let ve=0,Ee=oe.length;ve<Ee;ve++){const ge=r.get(oe[ve]);ge.__webglTexture&&(s.deleteTexture(ge.__webglTexture),c.memory.textures--),r.remove(oe[ve])}r.remove(F)}let w=0;function I(){w=0}function K(){const F=w;return F>=o.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+F+" texture units while this GPU supports only "+o.maxTextures),w+=1,F}function V(F){const A=[];return A.push(F.wrapS),A.push(F.wrapT),A.push(F.wrapR||0),A.push(F.magFilter),A.push(F.minFilter),A.push(F.anisotropy),A.push(F.internalFormat),A.push(F.format),A.push(F.type),A.push(F.generateMipmaps),A.push(F.premultiplyAlpha),A.push(F.flipY),A.push(F.unpackAlignment),A.push(F.colorSpace),A.join()}function Z(F,A){const oe=r.get(F);if(F.isVideoTexture&&De(F),F.isRenderTargetTexture===!1&&F.version>0&&oe.__version!==F.version){const ve=F.image;if(ve===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(ve.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{ke(oe,F,A);return}}t.bindTexture(s.TEXTURE_2D,oe.__webglTexture,s.TEXTURE0+A)}function te(F,A){const oe=r.get(F);if(F.version>0&&oe.__version!==F.version){ke(oe,F,A);return}t.bindTexture(s.TEXTURE_2D_ARRAY,oe.__webglTexture,s.TEXTURE0+A)}function ue(F,A){const oe=r.get(F);if(F.version>0&&oe.__version!==F.version){ke(oe,F,A);return}t.bindTexture(s.TEXTURE_3D,oe.__webglTexture,s.TEXTURE0+A)}function se(F,A){const oe=r.get(F);if(F.version>0&&oe.__version!==F.version){J(oe,F,A);return}t.bindTexture(s.TEXTURE_CUBE_MAP,oe.__webglTexture,s.TEXTURE0+A)}const G={[Wf]:s.REPEAT,[Kr]:s.CLAMP_TO_EDGE,[jf]:s.MIRRORED_REPEAT},he={[Kn]:s.NEAREST,[X0]:s.NEAREST_MIPMAP_NEAREST,[pl]:s.NEAREST_MIPMAP_LINEAR,[ci]:s.LINEAR,[ef]:s.LINEAR_MIPMAP_NEAREST,[Zr]:s.LINEAR_MIPMAP_LINEAR},fe={[K0]:s.NEVER,[nx]:s.ALWAYS,[Z0]:s.LESS,[Yg]:s.LEQUAL,[Q0]:s.EQUAL,[tx]:s.GEQUAL,[J0]:s.GREATER,[ex]:s.NOTEQUAL};function k(F,A){if(A.type===zi&&e.has("OES_texture_float_linear")===!1&&(A.magFilter===ci||A.magFilter===ef||A.magFilter===pl||A.magFilter===Zr||A.minFilter===ci||A.minFilter===ef||A.minFilter===pl||A.minFilter===Zr)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),s.texParameteri(F,s.TEXTURE_WRAP_S,G[A.wrapS]),s.texParameteri(F,s.TEXTURE_WRAP_T,G[A.wrapT]),(F===s.TEXTURE_3D||F===s.TEXTURE_2D_ARRAY)&&s.texParameteri(F,s.TEXTURE_WRAP_R,G[A.wrapR]),s.texParameteri(F,s.TEXTURE_MAG_FILTER,he[A.magFilter]),s.texParameteri(F,s.TEXTURE_MIN_FILTER,he[A.minFilter]),A.compareFunction&&(s.texParameteri(F,s.TEXTURE_COMPARE_MODE,s.COMPARE_REF_TO_TEXTURE),s.texParameteri(F,s.TEXTURE_COMPARE_FUNC,fe[A.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(A.magFilter===Kn||A.minFilter!==pl&&A.minFilter!==Zr||A.type===zi&&e.has("OES_texture_float_linear")===!1)return;if(A.anisotropy>1||r.get(A).__currentAnisotropy){const oe=e.get("EXT_texture_filter_anisotropic");s.texParameterf(F,oe.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(A.anisotropy,o.getMaxAnisotropy())),r.get(A).__currentAnisotropy=A.anisotropy}}}function ce(F,A){let oe=!1;F.__webglInit===void 0&&(F.__webglInit=!0,A.addEventListener("dispose",Y));const ve=A.source;let Ee=v.get(ve);Ee===void 0&&(Ee={},v.set(ve,Ee));const ge=V(A);if(ge!==F.__cacheKey){Ee[ge]===void 0&&(Ee[ge]={texture:s.createTexture(),usedTimes:0},c.memory.textures++,oe=!0),Ee[ge].usedTimes++;const Ze=Ee[F.__cacheKey];Ze!==void 0&&(Ee[F.__cacheKey].usedTimes--,Ze.usedTimes===0&&H(A)),F.__cacheKey=ge,F.__webglTexture=Ee[ge].texture}return oe}function ke(F,A,oe){let ve=s.TEXTURE_2D;(A.isDataArrayTexture||A.isCompressedArrayTexture)&&(ve=s.TEXTURE_2D_ARRAY),A.isData3DTexture&&(ve=s.TEXTURE_3D);const Ee=ce(F,A),ge=A.source;t.bindTexture(ve,F.__webglTexture,s.TEXTURE0+oe);const Ze=r.get(ge);if(ge.version!==Ze.__version||Ee===!0){t.activeTexture(s.TEXTURE0+oe);const Ne=Rt.getPrimaries(Rt.workingColorSpace),Be=A.colorSpace===vr?null:Rt.getPrimaries(A.colorSpace),ct=A.colorSpace===vr||Ne===Be?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,A.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,A.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,A.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,ct);let Te=S(A.image,!1,o.maxTextureSize);Te=Ae(A,Te);const Oe=u.convert(A.format,A.colorSpace),_t=u.convert(A.type);let rt=P(A.internalFormat,Oe,_t,A.colorSpace,A.isVideoTexture);k(ve,A);let Ve;const lt=A.mipmaps,ft=A.isVideoTexture!==!0,Ct=Ze.__version===void 0||Ee===!0,j=ge.dataReady,Ce=L(A,Te);if(A.isDepthTexture)rt=R(A.format===la,A.type),Ct&&(ft?t.texStorage2D(s.TEXTURE_2D,1,rt,Te.width,Te.height):t.texImage2D(s.TEXTURE_2D,0,rt,Te.width,Te.height,0,Oe,_t,null));else if(A.isDataTexture)if(lt.length>0){ft&&Ct&&t.texStorage2D(s.TEXTURE_2D,Ce,rt,lt[0].width,lt[0].height);for(let pe=0,me=lt.length;pe<me;pe++)Ve=lt[pe],ft?j&&t.texSubImage2D(s.TEXTURE_2D,pe,0,0,Ve.width,Ve.height,Oe,_t,Ve.data):t.texImage2D(s.TEXTURE_2D,pe,rt,Ve.width,Ve.height,0,Oe,_t,Ve.data);A.generateMipmaps=!1}else ft?(Ct&&t.texStorage2D(s.TEXTURE_2D,Ce,rt,Te.width,Te.height),j&&t.texSubImage2D(s.TEXTURE_2D,0,0,0,Te.width,Te.height,Oe,_t,Te.data)):t.texImage2D(s.TEXTURE_2D,0,rt,Te.width,Te.height,0,Oe,_t,Te.data);else if(A.isCompressedTexture)if(A.isCompressedArrayTexture){ft&&Ct&&t.texStorage3D(s.TEXTURE_2D_ARRAY,Ce,rt,lt[0].width,lt[0].height,Te.depth);for(let pe=0,me=lt.length;pe<me;pe++)if(Ve=lt[pe],A.format!==di)if(Oe!==null)if(ft){if(j)if(A.layerUpdates.size>0){const Re=dg(Ve.width,Ve.height,A.format,A.type);for(const Je of A.layerUpdates){const vt=Ve.data.subarray(Je*Re/Ve.data.BYTES_PER_ELEMENT,(Je+1)*Re/Ve.data.BYTES_PER_ELEMENT);t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,pe,0,0,Je,Ve.width,Ve.height,1,Oe,vt,0,0)}A.clearLayerUpdates()}else t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,pe,0,0,0,Ve.width,Ve.height,Te.depth,Oe,Ve.data,0,0)}else t.compressedTexImage3D(s.TEXTURE_2D_ARRAY,pe,rt,Ve.width,Ve.height,Te.depth,0,Ve.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else ft?j&&t.texSubImage3D(s.TEXTURE_2D_ARRAY,pe,0,0,0,Ve.width,Ve.height,Te.depth,Oe,_t,Ve.data):t.texImage3D(s.TEXTURE_2D_ARRAY,pe,rt,Ve.width,Ve.height,Te.depth,0,Oe,_t,Ve.data)}else{ft&&Ct&&t.texStorage2D(s.TEXTURE_2D,Ce,rt,lt[0].width,lt[0].height);for(let pe=0,me=lt.length;pe<me;pe++)Ve=lt[pe],A.format!==di?Oe!==null?ft?j&&t.compressedTexSubImage2D(s.TEXTURE_2D,pe,0,0,Ve.width,Ve.height,Oe,Ve.data):t.compressedTexImage2D(s.TEXTURE_2D,pe,rt,Ve.width,Ve.height,0,Ve.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):ft?j&&t.texSubImage2D(s.TEXTURE_2D,pe,0,0,Ve.width,Ve.height,Oe,_t,Ve.data):t.texImage2D(s.TEXTURE_2D,pe,rt,Ve.width,Ve.height,0,Oe,_t,Ve.data)}else if(A.isDataArrayTexture)if(ft){if(Ct&&t.texStorage3D(s.TEXTURE_2D_ARRAY,Ce,rt,Te.width,Te.height,Te.depth),j)if(A.layerUpdates.size>0){const pe=dg(Te.width,Te.height,A.format,A.type);for(const me of A.layerUpdates){const Re=Te.data.subarray(me*pe/Te.data.BYTES_PER_ELEMENT,(me+1)*pe/Te.data.BYTES_PER_ELEMENT);t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,me,Te.width,Te.height,1,Oe,_t,Re)}A.clearLayerUpdates()}else t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,0,Te.width,Te.height,Te.depth,Oe,_t,Te.data)}else t.texImage3D(s.TEXTURE_2D_ARRAY,0,rt,Te.width,Te.height,Te.depth,0,Oe,_t,Te.data);else if(A.isData3DTexture)ft?(Ct&&t.texStorage3D(s.TEXTURE_3D,Ce,rt,Te.width,Te.height,Te.depth),j&&t.texSubImage3D(s.TEXTURE_3D,0,0,0,0,Te.width,Te.height,Te.depth,Oe,_t,Te.data)):t.texImage3D(s.TEXTURE_3D,0,rt,Te.width,Te.height,Te.depth,0,Oe,_t,Te.data);else if(A.isFramebufferTexture){if(Ct)if(ft)t.texStorage2D(s.TEXTURE_2D,Ce,rt,Te.width,Te.height);else{let pe=Te.width,me=Te.height;for(let Re=0;Re<Ce;Re++)t.texImage2D(s.TEXTURE_2D,Re,rt,pe,me,0,Oe,_t,null),pe>>=1,me>>=1}}else if(lt.length>0){if(ft&&Ct){const pe=Ue(lt[0]);t.texStorage2D(s.TEXTURE_2D,Ce,rt,pe.width,pe.height)}for(let pe=0,me=lt.length;pe<me;pe++)Ve=lt[pe],ft?j&&t.texSubImage2D(s.TEXTURE_2D,pe,0,0,Oe,_t,Ve):t.texImage2D(s.TEXTURE_2D,pe,rt,Oe,_t,Ve);A.generateMipmaps=!1}else if(ft){if(Ct){const pe=Ue(Te);t.texStorage2D(s.TEXTURE_2D,Ce,rt,pe.width,pe.height)}j&&t.texSubImage2D(s.TEXTURE_2D,0,0,0,Oe,_t,Te)}else t.texImage2D(s.TEXTURE_2D,0,rt,Oe,_t,Te);x(A)&&_(ve),Ze.__version=ge.version,A.onUpdate&&A.onUpdate(A)}F.__version=A.version}function J(F,A,oe){if(A.image.length!==6)return;const ve=ce(F,A),Ee=A.source;t.bindTexture(s.TEXTURE_CUBE_MAP,F.__webglTexture,s.TEXTURE0+oe);const ge=r.get(Ee);if(Ee.version!==ge.__version||ve===!0){t.activeTexture(s.TEXTURE0+oe);const Ze=Rt.getPrimaries(Rt.workingColorSpace),Ne=A.colorSpace===vr?null:Rt.getPrimaries(A.colorSpace),Be=A.colorSpace===vr||Ze===Ne?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,A.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,A.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,A.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,Be);const ct=A.isCompressedTexture||A.image[0].isCompressedTexture,Te=A.image[0]&&A.image[0].isDataTexture,Oe=[];for(let me=0;me<6;me++)!ct&&!Te?Oe[me]=S(A.image[me],!0,o.maxCubemapSize):Oe[me]=Te?A.image[me].image:A.image[me],Oe[me]=Ae(A,Oe[me]);const _t=Oe[0],rt=u.convert(A.format,A.colorSpace),Ve=u.convert(A.type),lt=P(A.internalFormat,rt,Ve,A.colorSpace),ft=A.isVideoTexture!==!0,Ct=ge.__version===void 0||ve===!0,j=Ee.dataReady;let Ce=L(A,_t);k(s.TEXTURE_CUBE_MAP,A);let pe;if(ct){ft&&Ct&&t.texStorage2D(s.TEXTURE_CUBE_MAP,Ce,lt,_t.width,_t.height);for(let me=0;me<6;me++){pe=Oe[me].mipmaps;for(let Re=0;Re<pe.length;Re++){const Je=pe[Re];A.format!==di?rt!==null?ft?j&&t.compressedTexSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re,0,0,Je.width,Je.height,rt,Je.data):t.compressedTexImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re,lt,Je.width,Je.height,0,Je.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):ft?j&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re,0,0,Je.width,Je.height,rt,Ve,Je.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re,lt,Je.width,Je.height,0,rt,Ve,Je.data)}}}else{if(pe=A.mipmaps,ft&&Ct){pe.length>0&&Ce++;const me=Ue(Oe[0]);t.texStorage2D(s.TEXTURE_CUBE_MAP,Ce,lt,me.width,me.height)}for(let me=0;me<6;me++)if(Te){ft?j&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,0,0,0,Oe[me].width,Oe[me].height,rt,Ve,Oe[me].data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,0,lt,Oe[me].width,Oe[me].height,0,rt,Ve,Oe[me].data);for(let Re=0;Re<pe.length;Re++){const vt=pe[Re].image[me].image;ft?j&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re+1,0,0,vt.width,vt.height,rt,Ve,vt.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re+1,lt,vt.width,vt.height,0,rt,Ve,vt.data)}}else{ft?j&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,0,0,0,rt,Ve,Oe[me]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,0,lt,rt,Ve,Oe[me]);for(let Re=0;Re<pe.length;Re++){const Je=pe[Re];ft?j&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re+1,0,0,rt,Ve,Je.image[me]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+me,Re+1,lt,rt,Ve,Je.image[me])}}}x(A)&&_(s.TEXTURE_CUBE_MAP),ge.__version=Ee.version,A.onUpdate&&A.onUpdate(A)}F.__version=A.version}function de(F,A,oe,ve,Ee,ge){const Ze=u.convert(oe.format,oe.colorSpace),Ne=u.convert(oe.type),Be=P(oe.internalFormat,Ze,Ne,oe.colorSpace);if(!r.get(A).__hasExternalTextures){const Te=Math.max(1,A.width>>ge),Oe=Math.max(1,A.height>>ge);Ee===s.TEXTURE_3D||Ee===s.TEXTURE_2D_ARRAY?t.texImage3D(Ee,ge,Be,Te,Oe,A.depth,0,Ze,Ne,null):t.texImage2D(Ee,ge,Be,Te,Oe,0,Ze,Ne,null)}t.bindFramebuffer(s.FRAMEBUFFER,F),ie(A)?f.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,ve,Ee,r.get(oe).__webglTexture,0,ne(A)):(Ee===s.TEXTURE_2D||Ee>=s.TEXTURE_CUBE_MAP_POSITIVE_X&&Ee<=s.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&s.framebufferTexture2D(s.FRAMEBUFFER,ve,Ee,r.get(oe).__webglTexture,ge),t.bindFramebuffer(s.FRAMEBUFFER,null)}function Se(F,A,oe){if(s.bindRenderbuffer(s.RENDERBUFFER,F),A.depthBuffer){const ve=A.depthTexture,Ee=ve&&ve.isDepthTexture?ve.type:null,ge=R(A.stencilBuffer,Ee),Ze=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,Ne=ne(A);ie(A)?f.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,Ne,ge,A.width,A.height):oe?s.renderbufferStorageMultisample(s.RENDERBUFFER,Ne,ge,A.width,A.height):s.renderbufferStorage(s.RENDERBUFFER,ge,A.width,A.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,Ze,s.RENDERBUFFER,F)}else{const ve=A.textures;for(let Ee=0;Ee<ve.length;Ee++){const ge=ve[Ee],Ze=u.convert(ge.format,ge.colorSpace),Ne=u.convert(ge.type),Be=P(ge.internalFormat,Ze,Ne,ge.colorSpace),ct=ne(A);oe&&ie(A)===!1?s.renderbufferStorageMultisample(s.RENDERBUFFER,ct,Be,A.width,A.height):ie(A)?f.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,ct,Be,A.width,A.height):s.renderbufferStorage(s.RENDERBUFFER,Be,A.width,A.height)}}s.bindRenderbuffer(s.RENDERBUFFER,null)}function Me(F,A){if(A&&A.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(s.FRAMEBUFFER,F),!(A.depthTexture&&A.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!r.get(A.depthTexture).__webglTexture||A.depthTexture.image.width!==A.width||A.depthTexture.image.height!==A.height)&&(A.depthTexture.image.width=A.width,A.depthTexture.image.height=A.height,A.depthTexture.needsUpdate=!0),Z(A.depthTexture,0);const ve=r.get(A.depthTexture).__webglTexture,Ee=ne(A);if(A.depthTexture.format===ea)ie(A)?f.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,ve,0,Ee):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,ve,0);else if(A.depthTexture.format===la)ie(A)?f.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,ve,0,Ee):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,ve,0);else throw new Error("Unknown depthTexture format")}function Ie(F){const A=r.get(F),oe=F.isWebGLCubeRenderTarget===!0;if(F.depthTexture&&!A.__autoAllocateDepthBuffer){if(oe)throw new Error("target.depthTexture not supported in Cube render targets");Me(A.__webglFramebuffer,F)}else if(oe){A.__webglDepthbuffer=[];for(let ve=0;ve<6;ve++)t.bindFramebuffer(s.FRAMEBUFFER,A.__webglFramebuffer[ve]),A.__webglDepthbuffer[ve]=s.createRenderbuffer(),Se(A.__webglDepthbuffer[ve],F,!1)}else t.bindFramebuffer(s.FRAMEBUFFER,A.__webglFramebuffer),A.__webglDepthbuffer=s.createRenderbuffer(),Se(A.__webglDepthbuffer,F,!1);t.bindFramebuffer(s.FRAMEBUFFER,null)}function Ge(F,A,oe){const ve=r.get(F);A!==void 0&&de(ve.__webglFramebuffer,F,F.texture,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,0),oe!==void 0&&Ie(F)}function Qe(F){const A=F.texture,oe=r.get(F),ve=r.get(A);F.addEventListener("dispose",U);const Ee=F.textures,ge=F.isWebGLCubeRenderTarget===!0,Ze=Ee.length>1;if(Ze||(ve.__webglTexture===void 0&&(ve.__webglTexture=s.createTexture()),ve.__version=A.version,c.memory.textures++),ge){oe.__webglFramebuffer=[];for(let Ne=0;Ne<6;Ne++)if(A.mipmaps&&A.mipmaps.length>0){oe.__webglFramebuffer[Ne]=[];for(let Be=0;Be<A.mipmaps.length;Be++)oe.__webglFramebuffer[Ne][Be]=s.createFramebuffer()}else oe.__webglFramebuffer[Ne]=s.createFramebuffer()}else{if(A.mipmaps&&A.mipmaps.length>0){oe.__webglFramebuffer=[];for(let Ne=0;Ne<A.mipmaps.length;Ne++)oe.__webglFramebuffer[Ne]=s.createFramebuffer()}else oe.__webglFramebuffer=s.createFramebuffer();if(Ze)for(let Ne=0,Be=Ee.length;Ne<Be;Ne++){const ct=r.get(Ee[Ne]);ct.__webglTexture===void 0&&(ct.__webglTexture=s.createTexture(),c.memory.textures++)}if(F.samples>0&&ie(F)===!1){oe.__webglMultisampledFramebuffer=s.createFramebuffer(),oe.__webglColorRenderbuffer=[],t.bindFramebuffer(s.FRAMEBUFFER,oe.__webglMultisampledFramebuffer);for(let Ne=0;Ne<Ee.length;Ne++){const Be=Ee[Ne];oe.__webglColorRenderbuffer[Ne]=s.createRenderbuffer(),s.bindRenderbuffer(s.RENDERBUFFER,oe.__webglColorRenderbuffer[Ne]);const ct=u.convert(Be.format,Be.colorSpace),Te=u.convert(Be.type),Oe=P(Be.internalFormat,ct,Te,Be.colorSpace,F.isXRRenderTarget===!0),_t=ne(F);s.renderbufferStorageMultisample(s.RENDERBUFFER,_t,Oe,F.width,F.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+Ne,s.RENDERBUFFER,oe.__webglColorRenderbuffer[Ne])}s.bindRenderbuffer(s.RENDERBUFFER,null),F.depthBuffer&&(oe.__webglDepthRenderbuffer=s.createRenderbuffer(),Se(oe.__webglDepthRenderbuffer,F,!0)),t.bindFramebuffer(s.FRAMEBUFFER,null)}}if(ge){t.bindTexture(s.TEXTURE_CUBE_MAP,ve.__webglTexture),k(s.TEXTURE_CUBE_MAP,A);for(let Ne=0;Ne<6;Ne++)if(A.mipmaps&&A.mipmaps.length>0)for(let Be=0;Be<A.mipmaps.length;Be++)de(oe.__webglFramebuffer[Ne][Be],F,A,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+Ne,Be);else de(oe.__webglFramebuffer[Ne],F,A,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+Ne,0);x(A)&&_(s.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(Ze){for(let Ne=0,Be=Ee.length;Ne<Be;Ne++){const ct=Ee[Ne],Te=r.get(ct);t.bindTexture(s.TEXTURE_2D,Te.__webglTexture),k(s.TEXTURE_2D,ct),de(oe.__webglFramebuffer,F,ct,s.COLOR_ATTACHMENT0+Ne,s.TEXTURE_2D,0),x(ct)&&_(s.TEXTURE_2D)}t.unbindTexture()}else{let Ne=s.TEXTURE_2D;if((F.isWebGL3DRenderTarget||F.isWebGLArrayRenderTarget)&&(Ne=F.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),t.bindTexture(Ne,ve.__webglTexture),k(Ne,A),A.mipmaps&&A.mipmaps.length>0)for(let Be=0;Be<A.mipmaps.length;Be++)de(oe.__webglFramebuffer[Be],F,A,s.COLOR_ATTACHMENT0,Ne,Be);else de(oe.__webglFramebuffer,F,A,s.COLOR_ATTACHMENT0,Ne,0);x(A)&&_(Ne),t.unbindTexture()}F.depthBuffer&&Ie(F)}function Et(F){const A=F.textures;for(let oe=0,ve=A.length;oe<ve;oe++){const Ee=A[oe];if(x(Ee)){const ge=F.isWebGLCubeRenderTarget?s.TEXTURE_CUBE_MAP:s.TEXTURE_2D,Ze=r.get(Ee).__webglTexture;t.bindTexture(ge,Ze),_(ge),t.unbindTexture()}}}const B=[],wt=[];function gt(F){if(F.samples>0){if(ie(F)===!1){const A=F.textures,oe=F.width,ve=F.height;let Ee=s.COLOR_BUFFER_BIT;const ge=F.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,Ze=r.get(F),Ne=A.length>1;if(Ne)for(let Be=0;Be<A.length;Be++)t.bindFramebuffer(s.FRAMEBUFFER,Ze.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+Be,s.RENDERBUFFER,null),t.bindFramebuffer(s.FRAMEBUFFER,Ze.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+Be,s.TEXTURE_2D,null,0);t.bindFramebuffer(s.READ_FRAMEBUFFER,Ze.__webglMultisampledFramebuffer),t.bindFramebuffer(s.DRAW_FRAMEBUFFER,Ze.__webglFramebuffer);for(let Be=0;Be<A.length;Be++){if(F.resolveDepthBuffer&&(F.depthBuffer&&(Ee|=s.DEPTH_BUFFER_BIT),F.stencilBuffer&&F.resolveStencilBuffer&&(Ee|=s.STENCIL_BUFFER_BIT)),Ne){s.framebufferRenderbuffer(s.READ_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.RENDERBUFFER,Ze.__webglColorRenderbuffer[Be]);const ct=r.get(A[Be]).__webglTexture;s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,ct,0)}s.blitFramebuffer(0,0,oe,ve,0,0,oe,ve,Ee,s.NEAREST),h===!0&&(B.length=0,wt.length=0,B.push(s.COLOR_ATTACHMENT0+Be),F.depthBuffer&&F.resolveDepthBuffer===!1&&(B.push(ge),wt.push(ge),s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,wt)),s.invalidateFramebuffer(s.READ_FRAMEBUFFER,B))}if(t.bindFramebuffer(s.READ_FRAMEBUFFER,null),t.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),Ne)for(let Be=0;Be<A.length;Be++){t.bindFramebuffer(s.FRAMEBUFFER,Ze.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+Be,s.RENDERBUFFER,Ze.__webglColorRenderbuffer[Be]);const ct=r.get(A[Be]).__webglTexture;t.bindFramebuffer(s.FRAMEBUFFER,Ze.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+Be,s.TEXTURE_2D,ct,0)}t.bindFramebuffer(s.DRAW_FRAMEBUFFER,Ze.__webglMultisampledFramebuffer)}else if(F.depthBuffer&&F.resolveDepthBuffer===!1&&h){const A=F.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT;s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,[A])}}}function ne(F){return Math.min(o.maxSamples,F.samples)}function ie(F){const A=r.get(F);return F.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&A.__useRenderToTexture!==!1}function De(F){const A=c.render.frame;g.get(F)!==A&&(g.set(F,A),F.update())}function Ae(F,A){const oe=F.colorSpace,ve=F.format,Ee=F.type;return F.isCompressedTexture===!0||F.isVideoTexture===!0||oe!==wr&&oe!==vr&&(Rt.getTransfer(oe)===Ot?(ve!==di||Ee!==Vi)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",oe)),A}function Ue(F){return typeof HTMLImageElement<"u"&&F instanceof HTMLImageElement?(m.width=F.naturalWidth||F.width,m.height=F.naturalHeight||F.height):typeof VideoFrame<"u"&&F instanceof VideoFrame?(m.width=F.displayWidth,m.height=F.displayHeight):(m.width=F.width,m.height=F.height),m}this.allocateTextureUnit=K,this.resetTextureUnits=I,this.setTexture2D=Z,this.setTexture2DArray=te,this.setTexture3D=ue,this.setTextureCube=se,this.rebindTextures=Ge,this.setupRenderTarget=Qe,this.updateRenderTargetMipmap=Et,this.updateMultisampleRenderTarget=gt,this.setupDepthRenderbuffer=Ie,this.setupFrameBufferTexture=de,this.useMultisampledRTT=ie}function rw(s,e){function t(r,o=vr){let u;const c=Rt.getTransfer(o);if(r===Vi)return s.UNSIGNED_BYTE;if(r===wd)return s.UNSIGNED_SHORT_4_4_4_4;if(r===Td)return s.UNSIGNED_SHORT_5_5_5_1;if(r===kg)return s.UNSIGNED_INT_5_9_9_9_REV;if(r===Fg)return s.BYTE;if(r===Og)return s.SHORT;if(r===no)return s.UNSIGNED_SHORT;if(r===Ed)return s.INT;if(r===es)return s.UNSIGNED_INT;if(r===zi)return s.FLOAT;if(r===ro)return s.HALF_FLOAT;if(r===Bg)return s.ALPHA;if(r===zg)return s.RGB;if(r===di)return s.RGBA;if(r===Hg)return s.LUMINANCE;if(r===Vg)return s.LUMINANCE_ALPHA;if(r===ea)return s.DEPTH_COMPONENT;if(r===la)return s.DEPTH_STENCIL;if(r===Gg)return s.RED;if(r===Ad)return s.RED_INTEGER;if(r===Wg)return s.RG;if(r===Cd)return s.RG_INTEGER;if(r===Rd)return s.RGBA_INTEGER;if(r===ql||r===$l||r===Kl||r===Zl)if(c===Ot)if(u=e.get("WEBGL_compressed_texture_s3tc_srgb"),u!==null){if(r===ql)return u.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(r===$l)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(r===Kl)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(r===Zl)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(u=e.get("WEBGL_compressed_texture_s3tc"),u!==null){if(r===ql)return u.COMPRESSED_RGB_S3TC_DXT1_EXT;if(r===$l)return u.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(r===Kl)return u.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(r===Zl)return u.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(r===Xf||r===Yf||r===qf||r===$f)if(u=e.get("WEBGL_compressed_texture_pvrtc"),u!==null){if(r===Xf)return u.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(r===Yf)return u.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(r===qf)return u.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(r===$f)return u.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(r===Kf||r===Zf||r===Qf)if(u=e.get("WEBGL_compressed_texture_etc"),u!==null){if(r===Kf||r===Zf)return c===Ot?u.COMPRESSED_SRGB8_ETC2:u.COMPRESSED_RGB8_ETC2;if(r===Qf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:u.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(r===Jf||r===ed||r===td||r===nd||r===id||r===rd||r===sd||r===ad||r===od||r===ld||r===ud||r===cd||r===fd||r===dd)if(u=e.get("WEBGL_compressed_texture_astc"),u!==null){if(r===Jf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:u.COMPRESSED_RGBA_ASTC_4x4_KHR;if(r===ed)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:u.COMPRESSED_RGBA_ASTC_5x4_KHR;if(r===td)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:u.COMPRESSED_RGBA_ASTC_5x5_KHR;if(r===nd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:u.COMPRESSED_RGBA_ASTC_6x5_KHR;if(r===id)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:u.COMPRESSED_RGBA_ASTC_6x6_KHR;if(r===rd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:u.COMPRESSED_RGBA_ASTC_8x5_KHR;if(r===sd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:u.COMPRESSED_RGBA_ASTC_8x6_KHR;if(r===ad)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:u.COMPRESSED_RGBA_ASTC_8x8_KHR;if(r===od)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:u.COMPRESSED_RGBA_ASTC_10x5_KHR;if(r===ld)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:u.COMPRESSED_RGBA_ASTC_10x6_KHR;if(r===ud)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:u.COMPRESSED_RGBA_ASTC_10x8_KHR;if(r===cd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:u.COMPRESSED_RGBA_ASTC_10x10_KHR;if(r===fd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:u.COMPRESSED_RGBA_ASTC_12x10_KHR;if(r===dd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:u.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(r===Ql||r===hd||r===pd)if(u=e.get("EXT_texture_compression_bptc"),u!==null){if(r===Ql)return c===Ot?u.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:u.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(r===hd)return u.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(r===pd)return u.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(r===jg||r===md||r===gd||r===_d)if(u=e.get("EXT_texture_compression_rgtc"),u!==null){if(r===Ql)return u.COMPRESSED_RED_RGTC1_EXT;if(r===md)return u.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(r===gd)return u.COMPRESSED_RED_GREEN_RGTC2_EXT;if(r===_d)return u.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return r===oa?s.UNSIGNED_INT_24_8:s[r]!==void 0?s[r]:null}return{convert:t}}class sw extends $n{constructor(e=[]){super(),this.isArrayCamera=!0,this.cameras=e}}class Qr extends Qt{constructor(){super(),this.isGroup=!0,this.type="Group"}}const aw={type:"move"};class Rf{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Qr,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Qr,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new Q,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new Q),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Qr,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new Q,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new Q),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){const t=this._hand;if(t)for(const r of e.hand.values())this._getHandJoint(t,r)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,r){let o=null,u=null,c=null;const f=this._targetRay,h=this._grip,m=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(m&&e.hand){c=!0;for(const S of e.hand.values()){const x=t.getJointPose(S,r),_=this._getHandJoint(m,S);x!==null&&(_.matrix.fromArray(x.transform.matrix),_.matrix.decompose(_.position,_.rotation,_.scale),_.matrixWorldNeedsUpdate=!0,_.jointRadius=x.radius),_.visible=x!==null}const g=m.joints["index-finger-tip"],y=m.joints["thumb-tip"],v=g.position.distanceTo(y.position),M=.02,T=.005;m.inputState.pinching&&v>M+T?(m.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!m.inputState.pinching&&v<=M-T&&(m.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else h!==null&&e.gripSpace&&(u=t.getPose(e.gripSpace,r),u!==null&&(h.matrix.fromArray(u.transform.matrix),h.matrix.decompose(h.position,h.rotation,h.scale),h.matrixWorldNeedsUpdate=!0,u.linearVelocity?(h.hasLinearVelocity=!0,h.linearVelocity.copy(u.linearVelocity)):h.hasLinearVelocity=!1,u.angularVelocity?(h.hasAngularVelocity=!0,h.angularVelocity.copy(u.angularVelocity)):h.hasAngularVelocity=!1));f!==null&&(o=t.getPose(e.targetRaySpace,r),o===null&&u!==null&&(o=u),o!==null&&(f.matrix.fromArray(o.transform.matrix),f.matrix.decompose(f.position,f.rotation,f.scale),f.matrixWorldNeedsUpdate=!0,o.linearVelocity?(f.hasLinearVelocity=!0,f.linearVelocity.copy(o.linearVelocity)):f.hasLinearVelocity=!1,o.angularVelocity?(f.hasAngularVelocity=!0,f.angularVelocity.copy(o.angularVelocity)):f.hasAngularVelocity=!1,this.dispatchEvent(aw)))}return f!==null&&(f.visible=o!==null),h!==null&&(h.visible=u!==null),m!==null&&(m.visible=c!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){const r=new Qr;r.matrixAutoUpdate=!1,r.visible=!1,e.joints[t.jointName]=r,e.add(r)}return e.joints[t.jointName]}}const ow=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,lw=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`;class uw{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t,r){if(this.texture===null){const o=new Mn,u=e.properties.get(o);u.__webglTexture=t.texture,(t.depthNear!=r.depthNear||t.depthFar!=r.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=o}}getMesh(e){if(this.texture!==null&&this.mesh===null){const t=e.cameras[0].viewport,r=new Er({vertexShader:ow,fragmentShader:lw,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new zn(new hu(20,20),r)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}}class cw extends ca{constructor(e,t){super();const r=this;let o=null,u=1,c=null,f="local-floor",h=1,m=null,g=null,y=null,v=null,M=null,T=null;const S=new uw,x=t.getContextAttributes();let _=null,P=null;const R=[],L=[],Y=new ht;let U=null;const N=new $n;N.layers.enable(1),N.viewport=new Zt;const H=new $n;H.layers.enable(2),H.viewport=new Zt;const b=[N,H],w=new sw;w.layers.enable(1),w.layers.enable(2);let I=null,K=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(J){let de=R[J];return de===void 0&&(de=new Rf,R[J]=de),de.getTargetRaySpace()},this.getControllerGrip=function(J){let de=R[J];return de===void 0&&(de=new Rf,R[J]=de),de.getGripSpace()},this.getHand=function(J){let de=R[J];return de===void 0&&(de=new Rf,R[J]=de),de.getHandSpace()};function V(J){const de=L.indexOf(J.inputSource);if(de===-1)return;const Se=R[de];Se!==void 0&&(Se.update(J.inputSource,J.frame,m||c),Se.dispatchEvent({type:J.type,data:J.inputSource}))}function Z(){o.removeEventListener("select",V),o.removeEventListener("selectstart",V),o.removeEventListener("selectend",V),o.removeEventListener("squeeze",V),o.removeEventListener("squeezestart",V),o.removeEventListener("squeezeend",V),o.removeEventListener("end",Z),o.removeEventListener("inputsourceschange",te);for(let J=0;J<R.length;J++){const de=L[J];de!==null&&(L[J]=null,R[J].disconnect(de))}I=null,K=null,S.reset(),e.setRenderTarget(_),M=null,v=null,y=null,o=null,P=null,ke.stop(),r.isPresenting=!1,e.setPixelRatio(U),e.setSize(Y.width,Y.height,!1),r.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(J){u=J,r.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(J){f=J,r.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return m||c},this.setReferenceSpace=function(J){m=J},this.getBaseLayer=function(){return v!==null?v:M},this.getBinding=function(){return y},this.getFrame=function(){return T},this.getSession=function(){return o},this.setSession=async function(J){if(o=J,o!==null){if(_=e.getRenderTarget(),o.addEventListener("select",V),o.addEventListener("selectstart",V),o.addEventListener("selectend",V),o.addEventListener("squeeze",V),o.addEventListener("squeezestart",V),o.addEventListener("squeezeend",V),o.addEventListener("end",Z),o.addEventListener("inputsourceschange",te),x.xrCompatible!==!0&&await t.makeXRCompatible(),U=e.getPixelRatio(),e.getSize(Y),o.renderState.layers===void 0){const de={antialias:x.antialias,alpha:!0,depth:x.depth,stencil:x.stencil,framebufferScaleFactor:u};M=new XRWebGLLayer(o,t,de),o.updateRenderState({baseLayer:M}),e.setPixelRatio(1),e.setSize(M.framebufferWidth,M.framebufferHeight,!1),P=new ts(M.framebufferWidth,M.framebufferHeight,{format:di,type:Vi,colorSpace:e.outputColorSpace,stencilBuffer:x.stencil})}else{let de=null,Se=null,Me=null;x.depth&&(Me=x.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,de=x.stencil?la:ea,Se=x.stencil?oa:es);const Ie={colorFormat:t.RGBA8,depthFormat:Me,scaleFactor:u};y=new XRWebGLBinding(o,t),v=y.createProjectionLayer(Ie),o.updateRenderState({layers:[v]}),e.setPixelRatio(1),e.setSize(v.textureWidth,v.textureHeight,!1),P=new ts(v.textureWidth,v.textureHeight,{format:di,type:Vi,depthTexture:new a_(v.textureWidth,v.textureHeight,Se,void 0,void 0,void 0,void 0,void 0,void 0,de),stencilBuffer:x.stencil,colorSpace:e.outputColorSpace,samples:x.antialias?4:0,resolveDepthBuffer:v.ignoreDepthValues===!1})}P.isXRRenderTarget=!0,this.setFoveation(h),m=null,c=await o.requestReferenceSpace(f),ke.setContext(o),ke.start(),r.isPresenting=!0,r.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(o!==null)return o.environmentBlendMode},this.getDepthTexture=function(){return S.getDepthTexture()};function te(J){for(let de=0;de<J.removed.length;de++){const Se=J.removed[de],Me=L.indexOf(Se);Me>=0&&(L[Me]=null,R[Me].disconnect(Se))}for(let de=0;de<J.added.length;de++){const Se=J.added[de];let Me=L.indexOf(Se);if(Me===-1){for(let Ge=0;Ge<R.length;Ge++)if(Ge>=L.length){L.push(Se),Me=Ge;break}else if(L[Ge]===null){L[Ge]=Se,Me=Ge;break}if(Me===-1)break}const Ie=R[Me];Ie&&Ie.connect(Se)}}const ue=new Q,se=new Q;function G(J,de,Se){ue.setFromMatrixPosition(de.matrixWorld),se.setFromMatrixPosition(Se.matrixWorld);const Me=ue.distanceTo(se),Ie=de.projectionMatrix.elements,Ge=Se.projectionMatrix.elements,Qe=Ie[14]/(Ie[10]-1),Et=Ie[14]/(Ie[10]+1),B=(Ie[9]+1)/Ie[5],wt=(Ie[9]-1)/Ie[5],gt=(Ie[8]-1)/Ie[0],ne=(Ge[8]+1)/Ge[0],ie=Qe*gt,De=Qe*ne,Ae=Me/(-gt+ne),Ue=Ae*-gt;de.matrixWorld.decompose(J.position,J.quaternion,J.scale),J.translateX(Ue),J.translateZ(Ae),J.matrixWorld.compose(J.position,J.quaternion,J.scale),J.matrixWorldInverse.copy(J.matrixWorld).invert();const F=Qe+Ae,A=Et+Ae,oe=ie-Ue,ve=De+(Me-Ue),Ee=B*Et/A*F,ge=wt*Et/A*F;J.projectionMatrix.makePerspective(oe,ve,Ee,ge,F,A),J.projectionMatrixInverse.copy(J.projectionMatrix).invert()}function he(J,de){de===null?J.matrixWorld.copy(J.matrix):J.matrixWorld.multiplyMatrices(de.matrixWorld,J.matrix),J.matrixWorldInverse.copy(J.matrixWorld).invert()}this.updateCamera=function(J){if(o===null)return;S.texture!==null&&(J.near=S.depthNear,J.far=S.depthFar),w.near=H.near=N.near=J.near,w.far=H.far=N.far=J.far,(I!==w.near||K!==w.far)&&(o.updateRenderState({depthNear:w.near,depthFar:w.far}),I=w.near,K=w.far,N.near=I,N.far=K,H.near=I,H.far=K,N.updateProjectionMatrix(),H.updateProjectionMatrix(),J.updateProjectionMatrix());const de=J.parent,Se=w.cameras;he(w,de);for(let Me=0;Me<Se.length;Me++)he(Se[Me],de);Se.length===2?G(w,N,H):w.projectionMatrix.copy(N.projectionMatrix),fe(J,w,de)};function fe(J,de,Se){Se===null?J.matrix.copy(de.matrixWorld):(J.matrix.copy(Se.matrixWorld),J.matrix.invert(),J.matrix.multiply(de.matrixWorld)),J.matrix.decompose(J.position,J.quaternion,J.scale),J.updateMatrixWorld(!0),J.projectionMatrix.copy(de.projectionMatrix),J.projectionMatrixInverse.copy(de.projectionMatrixInverse),J.isPerspectiveCamera&&(J.fov=xd*2*Math.atan(1/J.projectionMatrix.elements[5]),J.zoom=1)}this.getCamera=function(){return w},this.getFoveation=function(){if(!(v===null&&M===null))return h},this.setFoveation=function(J){h=J,v!==null&&(v.fixedFoveation=J),M!==null&&M.fixedFoveation!==void 0&&(M.fixedFoveation=J)},this.hasDepthSensing=function(){return S.texture!==null},this.getDepthSensingMesh=function(){return S.getMesh(w)};let k=null;function ce(J,de){if(g=de.getViewerPose(m||c),T=de,g!==null){const Se=g.views;M!==null&&(e.setRenderTargetFramebuffer(P,M.framebuffer),e.setRenderTarget(P));let Me=!1;Se.length!==w.cameras.length&&(w.cameras.length=0,Me=!0);for(let Ge=0;Ge<Se.length;Ge++){const Qe=Se[Ge];let Et=null;if(M!==null)Et=M.getViewport(Qe);else{const wt=y.getViewSubImage(v,Qe);Et=wt.viewport,Ge===0&&(e.setRenderTargetTextures(P,wt.colorTexture,v.ignoreDepthValues?void 0:wt.depthStencilTexture),e.setRenderTarget(P))}let B=b[Ge];B===void 0&&(B=new $n,B.layers.enable(Ge),B.viewport=new Zt,b[Ge]=B),B.matrix.fromArray(Qe.transform.matrix),B.matrix.decompose(B.position,B.quaternion,B.scale),B.projectionMatrix.fromArray(Qe.projectionMatrix),B.projectionMatrixInverse.copy(B.projectionMatrix).invert(),B.viewport.set(Et.x,Et.y,Et.width,Et.height),Ge===0&&(w.matrix.copy(B.matrix),w.matrix.decompose(w.position,w.quaternion,w.scale)),Me===!0&&w.cameras.push(B)}const Ie=o.enabledFeatures;if(Ie&&Ie.includes("depth-sensing")){const Ge=y.getDepthInformation(Se[0]);Ge&&Ge.isValid&&Ge.texture&&S.init(e,Ge,o.renderState)}}for(let Se=0;Se<R.length;Se++){const Me=L[Se],Ie=R[Se];Me!==null&&Ie!==void 0&&Ie.update(Me,de,m||c)}k&&k(J,de),de.detectedPlanes&&r.dispatchEvent({type:"planesdetected",data:de}),T=null}const ke=new s_;ke.setAnimationLoop(ce),this.setAnimationLoop=function(J){k=J},this.dispose=function(){}}}const Wr=new Si,fw=new Gt;function dw(s,e){function t(x,_){x.matrixAutoUpdate===!0&&x.updateMatrix(),_.value.copy(x.matrix)}function r(x,_){_.color.getRGB(x.fogColor.value,n_(s)),_.isFog?(x.fogNear.value=_.near,x.fogFar.value=_.far):_.isFogExp2&&(x.fogDensity.value=_.density)}function o(x,_,P,R,L){_.isMeshBasicMaterial||_.isMeshLambertMaterial?u(x,_):_.isMeshToonMaterial?(u(x,_),y(x,_)):_.isMeshPhongMaterial?(u(x,_),g(x,_)):_.isMeshStandardMaterial?(u(x,_),v(x,_),_.isMeshPhysicalMaterial&&M(x,_,L)):_.isMeshMatcapMaterial?(u(x,_),T(x,_)):_.isMeshDepthMaterial?u(x,_):_.isMeshDistanceMaterial?(u(x,_),S(x,_)):_.isMeshNormalMaterial?u(x,_):_.isLineBasicMaterial?(c(x,_),_.isLineDashedMaterial&&f(x,_)):_.isPointsMaterial?h(x,_,P,R):_.isSpriteMaterial?m(x,_):_.isShadowMaterial?(x.color.value.copy(_.color),x.opacity.value=_.opacity):_.isShaderMaterial&&(_.uniformsNeedUpdate=!1)}function u(x,_){x.opacity.value=_.opacity,_.color&&x.diffuse.value.copy(_.color),_.emissive&&x.emissive.value.copy(_.emissive).multiplyScalar(_.emissiveIntensity),_.map&&(x.map.value=_.map,t(_.map,x.mapTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.bumpMap&&(x.bumpMap.value=_.bumpMap,t(_.bumpMap,x.bumpMapTransform),x.bumpScale.value=_.bumpScale,_.side===Ln&&(x.bumpScale.value*=-1)),_.normalMap&&(x.normalMap.value=_.normalMap,t(_.normalMap,x.normalMapTransform),x.normalScale.value.copy(_.normalScale),_.side===Ln&&x.normalScale.value.negate()),_.displacementMap&&(x.displacementMap.value=_.displacementMap,t(_.displacementMap,x.displacementMapTransform),x.displacementScale.value=_.displacementScale,x.displacementBias.value=_.displacementBias),_.emissiveMap&&(x.emissiveMap.value=_.emissiveMap,t(_.emissiveMap,x.emissiveMapTransform)),_.specularMap&&(x.specularMap.value=_.specularMap,t(_.specularMap,x.specularMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest);const P=e.get(_),R=P.envMap,L=P.envMapRotation;R&&(x.envMap.value=R,Wr.copy(L),Wr.x*=-1,Wr.y*=-1,Wr.z*=-1,R.isCubeTexture&&R.isRenderTargetTexture===!1&&(Wr.y*=-1,Wr.z*=-1),x.envMapRotation.value.setFromMatrix4(fw.makeRotationFromEuler(Wr)),x.flipEnvMap.value=R.isCubeTexture&&R.isRenderTargetTexture===!1?-1:1,x.reflectivity.value=_.reflectivity,x.ior.value=_.ior,x.refractionRatio.value=_.refractionRatio),_.lightMap&&(x.lightMap.value=_.lightMap,x.lightMapIntensity.value=_.lightMapIntensity,t(_.lightMap,x.lightMapTransform)),_.aoMap&&(x.aoMap.value=_.aoMap,x.aoMapIntensity.value=_.aoMapIntensity,t(_.aoMap,x.aoMapTransform))}function c(x,_){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,_.map&&(x.map.value=_.map,t(_.map,x.mapTransform))}function f(x,_){x.dashSize.value=_.dashSize,x.totalSize.value=_.dashSize+_.gapSize,x.scale.value=_.scale}function h(x,_,P,R){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,x.size.value=_.size*P,x.scale.value=R*.5,_.map&&(x.map.value=_.map,t(_.map,x.uvTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest)}function m(x,_){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,x.rotation.value=_.rotation,_.map&&(x.map.value=_.map,t(_.map,x.mapTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest)}function g(x,_){x.specular.value.copy(_.specular),x.shininess.value=Math.max(_.shininess,1e-4)}function y(x,_){_.gradientMap&&(x.gradientMap.value=_.gradientMap)}function v(x,_){x.metalness.value=_.metalness,_.metalnessMap&&(x.metalnessMap.value=_.metalnessMap,t(_.metalnessMap,x.metalnessMapTransform)),x.roughness.value=_.roughness,_.roughnessMap&&(x.roughnessMap.value=_.roughnessMap,t(_.roughnessMap,x.roughnessMapTransform)),_.envMap&&(x.envMapIntensity.value=_.envMapIntensity)}function M(x,_,P){x.ior.value=_.ior,_.sheen>0&&(x.sheenColor.value.copy(_.sheenColor).multiplyScalar(_.sheen),x.sheenRoughness.value=_.sheenRoughness,_.sheenColorMap&&(x.sheenColorMap.value=_.sheenColorMap,t(_.sheenColorMap,x.sheenColorMapTransform)),_.sheenRoughnessMap&&(x.sheenRoughnessMap.value=_.sheenRoughnessMap,t(_.sheenRoughnessMap,x.sheenRoughnessMapTransform))),_.clearcoat>0&&(x.clearcoat.value=_.clearcoat,x.clearcoatRoughness.value=_.clearcoatRoughness,_.clearcoatMap&&(x.clearcoatMap.value=_.clearcoatMap,t(_.clearcoatMap,x.clearcoatMapTransform)),_.clearcoatRoughnessMap&&(x.clearcoatRoughnessMap.value=_.clearcoatRoughnessMap,t(_.clearcoatRoughnessMap,x.clearcoatRoughnessMapTransform)),_.clearcoatNormalMap&&(x.clearcoatNormalMap.value=_.clearcoatNormalMap,t(_.clearcoatNormalMap,x.clearcoatNormalMapTransform),x.clearcoatNormalScale.value.copy(_.clearcoatNormalScale),_.side===Ln&&x.clearcoatNormalScale.value.negate())),_.dispersion>0&&(x.dispersion.value=_.dispersion),_.iridescence>0&&(x.iridescence.value=_.iridescence,x.iridescenceIOR.value=_.iridescenceIOR,x.iridescenceThicknessMinimum.value=_.iridescenceThicknessRange[0],x.iridescenceThicknessMaximum.value=_.iridescenceThicknessRange[1],_.iridescenceMap&&(x.iridescenceMap.value=_.iridescenceMap,t(_.iridescenceMap,x.iridescenceMapTransform)),_.iridescenceThicknessMap&&(x.iridescenceThicknessMap.value=_.iridescenceThicknessMap,t(_.iridescenceThicknessMap,x.iridescenceThicknessMapTransform))),_.transmission>0&&(x.transmission.value=_.transmission,x.transmissionSamplerMap.value=P.texture,x.transmissionSamplerSize.value.set(P.width,P.height),_.transmissionMap&&(x.transmissionMap.value=_.transmissionMap,t(_.transmissionMap,x.transmissionMapTransform)),x.thickness.value=_.thickness,_.thicknessMap&&(x.thicknessMap.value=_.thicknessMap,t(_.thicknessMap,x.thicknessMapTransform)),x.attenuationDistance.value=_.attenuationDistance,x.attenuationColor.value.copy(_.attenuationColor)),_.anisotropy>0&&(x.anisotropyVector.value.set(_.anisotropy*Math.cos(_.anisotropyRotation),_.anisotropy*Math.sin(_.anisotropyRotation)),_.anisotropyMap&&(x.anisotropyMap.value=_.anisotropyMap,t(_.anisotropyMap,x.anisotropyMapTransform))),x.specularIntensity.value=_.specularIntensity,x.specularColor.value.copy(_.specularColor),_.specularColorMap&&(x.specularColorMap.value=_.specularColorMap,t(_.specularColorMap,x.specularColorMapTransform)),_.specularIntensityMap&&(x.specularIntensityMap.value=_.specularIntensityMap,t(_.specularIntensityMap,x.specularIntensityMapTransform))}function T(x,_){_.matcap&&(x.matcap.value=_.matcap)}function S(x,_){const P=e.get(_).light;x.referencePosition.value.setFromMatrixPosition(P.matrixWorld),x.nearDistance.value=P.shadow.camera.near,x.farDistance.value=P.shadow.camera.far}return{refreshFogUniforms:r,refreshMaterialUniforms:o}}function hw(s,e,t,r){let o={},u={},c=[];const f=s.getParameter(s.MAX_UNIFORM_BUFFER_BINDINGS);function h(P,R){const L=R.program;r.uniformBlockBinding(P,L)}function m(P,R){let L=o[P.id];L===void 0&&(T(P),L=g(P),o[P.id]=L,P.addEventListener("dispose",x));const Y=R.program;r.updateUBOMapping(P,Y);const U=e.render.frame;u[P.id]!==U&&(v(P),u[P.id]=U)}function g(P){const R=y();P.__bindingPointIndex=R;const L=s.createBuffer(),Y=P.__size,U=P.usage;return s.bindBuffer(s.UNIFORM_BUFFER,L),s.bufferData(s.UNIFORM_BUFFER,Y,U),s.bindBuffer(s.UNIFORM_BUFFER,null),s.bindBufferBase(s.UNIFORM_BUFFER,R,L),L}function y(){for(let P=0;P<f;P++)if(c.indexOf(P)===-1)return c.push(P),P;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function v(P){const R=o[P.id],L=P.uniforms,Y=P.__cache;s.bindBuffer(s.UNIFORM_BUFFER,R);for(let U=0,N=L.length;U<N;U++){const H=Array.isArray(L[U])?L[U]:[L[U]];for(let b=0,w=H.length;b<w;b++){const I=H[b];if(M(I,U,b,Y)===!0){const K=I.__offset,V=Array.isArray(I.value)?I.value:[I.value];let Z=0;for(let te=0;te<V.length;te++){const ue=V[te],se=S(ue);typeof ue=="number"||typeof ue=="boolean"?(I.__data[0]=ue,s.bufferSubData(s.UNIFORM_BUFFER,K+Z,I.__data)):ue.isMatrix3?(I.__data[0]=ue.elements[0],I.__data[1]=ue.elements[1],I.__data[2]=ue.elements[2],I.__data[3]=0,I.__data[4]=ue.elements[3],I.__data[5]=ue.elements[4],I.__data[6]=ue.elements[5],I.__data[7]=0,I.__data[8]=ue.elements[6],I.__data[9]=ue.elements[7],I.__data[10]=ue.elements[8],I.__data[11]=0):(ue.toArray(I.__data,Z),Z+=se.storage/Float32Array.BYTES_PER_ELEMENT)}s.bufferSubData(s.UNIFORM_BUFFER,K,I.__data)}}}s.bindBuffer(s.UNIFORM_BUFFER,null)}function M(P,R,L,Y){const U=P.value,N=R+"_"+L;if(Y[N]===void 0)return typeof U=="number"||typeof U=="boolean"?Y[N]=U:Y[N]=U.clone(),!0;{const H=Y[N];if(typeof U=="number"||typeof U=="boolean"){if(H!==U)return Y[N]=U,!0}else if(H.equals(U)===!1)return H.copy(U),!0}return!1}function T(P){const R=P.uniforms;let L=0;const Y=16;for(let N=0,H=R.length;N<H;N++){const b=Array.isArray(R[N])?R[N]:[R[N]];for(let w=0,I=b.length;w<I;w++){const K=b[w],V=Array.isArray(K.value)?K.value:[K.value];for(let Z=0,te=V.length;Z<te;Z++){const ue=V[Z],se=S(ue),G=L%Y,he=G%se.boundary,fe=G+he;L+=he,fe!==0&&Y-fe<se.storage&&(L+=Y-fe),K.__data=new Float32Array(se.storage/Float32Array.BYTES_PER_ELEMENT),K.__offset=L,L+=se.storage}}}const U=L%Y;return U>0&&(L+=Y-U),P.__size=L,P.__cache={},this}function S(P){const R={boundary:0,storage:0};return typeof P=="number"||typeof P=="boolean"?(R.boundary=4,R.storage=4):P.isVector2?(R.boundary=8,R.storage=8):P.isVector3||P.isColor?(R.boundary=16,R.storage=12):P.isVector4?(R.boundary=16,R.storage=16):P.isMatrix3?(R.boundary=48,R.storage=48):P.isMatrix4?(R.boundary=64,R.storage=64):P.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",P),R}function x(P){const R=P.target;R.removeEventListener("dispose",x);const L=c.indexOf(R.__bindingPointIndex);c.splice(L,1),s.deleteBuffer(o[R.id]),delete o[R.id],delete u[R.id]}function _(){for(const P in o)s.deleteBuffer(o[P]);c=[],o={},u={}}return{bind:h,update:m,dispose:_}}class pw{constructor(e={}){const{canvas:t=rx(),context:r=null,depth:o=!0,stencil:u=!1,alpha:c=!1,antialias:f=!1,premultipliedAlpha:h=!0,preserveDrawingBuffer:m=!1,powerPreference:g="default",failIfMajorPerformanceCaveat:y=!1}=e;this.isWebGLRenderer=!0;let v;if(r!==null){if(typeof WebGLRenderingContext<"u"&&r instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");v=r.getContextAttributes().alpha}else v=c;const M=new Uint32Array(4),T=new Int32Array(4);let S=null,x=null;const _=[],P=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=ui,this.toneMapping=yr,this.toneMappingExposure=1;const R=this;let L=!1,Y=0,U=0,N=null,H=-1,b=null;const w=new Zt,I=new Zt;let K=null;const V=new yt(0);let Z=0,te=t.width,ue=t.height,se=1,G=null,he=null;const fe=new Zt(0,0,te,ue),k=new Zt(0,0,te,ue);let ce=!1;const ke=new Ld;let J=!1,de=!1;const Se=new Gt,Me=new Q,Ie=new Zt,Ge={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let Qe=!1;function Et(){return N===null?se:1}let B=r;function wt(C,X){return t.getContext(C,X)}try{const C={alpha:!0,depth:o,stencil:u,antialias:f,premultipliedAlpha:h,preserveDrawingBuffer:m,powerPreference:g,failIfMajorPerformanceCaveat:y};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${Md}`),t.addEventListener("webglcontextlost",pe,!1),t.addEventListener("webglcontextrestored",me,!1),t.addEventListener("webglcontextcreationerror",Re,!1),B===null){const X="webgl2";if(B=wt(X,C),B===null)throw wt(X)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(C){throw console.error("THREE.WebGLRenderer: "+C.message),C}let gt,ne,ie,De,Ae,Ue,F,A,oe,ve,Ee,ge,Ze,Ne,Be,ct,Te,Oe,_t,rt,Ve,lt,ft,Ct;function j(){gt=new yM(B),gt.init(),lt=new rw(B,gt),ne=new pM(B,gt,e,lt),ie=new tw(B),De=new EM(B),Ae=new HE,Ue=new iw(B,gt,ie,Ae,ne,lt,De),F=new gM(R),A=new xM(R),oe=new Px(B),ft=new dM(B,oe),ve=new SM(B,oe,De,ft),Ee=new TM(B,ve,oe,De),_t=new wM(B,ne,Ue),ct=new mM(Ae),ge=new zE(R,F,A,gt,ne,ft,ct),Ze=new dw(R,Ae),Ne=new GE,Be=new $E(gt),Oe=new fM(R,F,A,ie,Ee,v,h),Te=new ew(R,Ee,ne),Ct=new hw(B,De,ne,ie),rt=new hM(B,gt,De),Ve=new MM(B,gt,De),De.programs=ge.programs,R.capabilities=ne,R.extensions=gt,R.properties=Ae,R.renderLists=Ne,R.shadowMap=Te,R.state=ie,R.info=De}j();const Ce=new cw(R,B);this.xr=Ce,this.getContext=function(){return B},this.getContextAttributes=function(){return B.getContextAttributes()},this.forceContextLoss=function(){const C=gt.get("WEBGL_lose_context");C&&C.loseContext()},this.forceContextRestore=function(){const C=gt.get("WEBGL_lose_context");C&&C.restoreContext()},this.getPixelRatio=function(){return se},this.setPixelRatio=function(C){C!==void 0&&(se=C,this.setSize(te,ue,!1))},this.getSize=function(C){return C.set(te,ue)},this.setSize=function(C,X,re=!0){if(Ce.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}te=C,ue=X,t.width=Math.floor(C*se),t.height=Math.floor(X*se),re===!0&&(t.style.width=C+"px",t.style.height=X+"px"),this.setViewport(0,0,C,X)},this.getDrawingBufferSize=function(C){return C.set(te*se,ue*se).floor()},this.setDrawingBufferSize=function(C,X,re){te=C,ue=X,se=re,t.width=Math.floor(C*re),t.height=Math.floor(X*re),this.setViewport(0,0,C,X)},this.getCurrentViewport=function(C){return C.copy(w)},this.getViewport=function(C){return C.copy(fe)},this.setViewport=function(C,X,re,le){C.isVector4?fe.set(C.x,C.y,C.z,C.w):fe.set(C,X,re,le),ie.viewport(w.copy(fe).multiplyScalar(se).round())},this.getScissor=function(C){return C.copy(k)},this.setScissor=function(C,X,re,le){C.isVector4?k.set(C.x,C.y,C.z,C.w):k.set(C,X,re,le),ie.scissor(I.copy(k).multiplyScalar(se).round())},this.getScissorTest=function(){return ce},this.setScissorTest=function(C){ie.setScissorTest(ce=C)},this.setOpaqueSort=function(C){G=C},this.setTransparentSort=function(C){he=C},this.getClearColor=function(C){return C.copy(Oe.getClearColor())},this.setClearColor=function(){Oe.setClearColor.apply(Oe,arguments)},this.getClearAlpha=function(){return Oe.getClearAlpha()},this.setClearAlpha=function(){Oe.setClearAlpha.apply(Oe,arguments)},this.clear=function(C=!0,X=!0,re=!0){let le=0;if(C){let q=!1;if(N!==null){const Pe=N.texture.format;q=Pe===Rd||Pe===Cd||Pe===Ad}if(q){const Pe=N.texture.type,ze=Pe===Vi||Pe===es||Pe===no||Pe===oa||Pe===wd||Pe===Td,Xe=Oe.getClearColor(),Le=Oe.getClearAlpha(),ot=Xe.r,st=Xe.g,et=Xe.b;ze?(M[0]=ot,M[1]=st,M[2]=et,M[3]=Le,B.clearBufferuiv(B.COLOR,0,M)):(T[0]=ot,T[1]=st,T[2]=et,T[3]=Le,B.clearBufferiv(B.COLOR,0,T))}else le|=B.COLOR_BUFFER_BIT}X&&(le|=B.DEPTH_BUFFER_BIT),re&&(le|=B.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),B.clear(le)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",pe,!1),t.removeEventListener("webglcontextrestored",me,!1),t.removeEventListener("webglcontextcreationerror",Re,!1),Ne.dispose(),Be.dispose(),Ae.dispose(),F.dispose(),A.dispose(),Ee.dispose(),ft.dispose(),Ct.dispose(),ge.dispose(),Ce.dispose(),Ce.removeEventListener("sessionstart",qt),Ce.removeEventListener("sessionend",En),Qn.stop()};function pe(C){C.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),L=!0}function me(){console.log("THREE.WebGLRenderer: Context Restored."),L=!1;const C=De.autoReset,X=Te.enabled,re=Te.autoUpdate,le=Te.needsUpdate,q=Te.type;j(),De.autoReset=C,Te.enabled=X,Te.autoUpdate=re,Te.needsUpdate=le,Te.type=q}function Re(C){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",C.statusMessage)}function Je(C){const X=C.target;X.removeEventListener("dispose",Je),vt(X)}function vt(C){xe(C),Ae.remove(C)}function xe(C){const X=Ae.get(C).programs;X!==void 0&&(X.forEach(function(re){ge.releaseProgram(re)}),C.isShaderMaterial&&ge.releaseShaderCache(C))}this.renderBufferDirect=function(C,X,re,le,q,Pe){X===null&&(X=Ge);const ze=q.isMesh&&q.matrixWorld.determinant()<0,Xe=_u(C,X,re,le,q);ie.setMaterial(le,ze);let Le=re.index,ot=1;if(le.wireframe===!0){if(Le=ve.getWireframeAttribute(re),Le===void 0)return;ot=2}const st=re.drawRange,et=re.attributes.position;let St=st.start*ot,Bt=(st.start+st.count)*ot;Pe!==null&&(St=Math.max(St,Pe.start*ot),Bt=Math.min(Bt,(Pe.start+Pe.count)*ot)),Le!==null?(St=Math.max(St,0),Bt=Math.min(Bt,Le.count)):et!=null&&(St=Math.max(St,0),Bt=Math.min(Bt,et.count));const It=Bt-St;if(It<0||It===1/0)return;ft.setup(q,le,Xe,re,Le);let tn,dt=rt;if(Le!==null&&(tn=oe.get(Le),dt=Ve,dt.setIndex(tn)),q.isMesh)le.wireframe===!0?(ie.setLineWidth(le.wireframeLinewidth*Et()),dt.setMode(B.LINES)):dt.setMode(B.TRIANGLES);else if(q.isLine){let $e=le.linewidth;$e===void 0&&($e=1),ie.setLineWidth($e*Et()),q.isLineSegments?dt.setMode(B.LINES):q.isLineLoop?dt.setMode(B.LINE_LOOP):dt.setMode(B.LINE_STRIP)}else q.isPoints?dt.setMode(B.POINTS):q.isSprite&&dt.setMode(B.TRIANGLES);if(q.isBatchedMesh)if(q._multiDrawInstances!==null)dt.renderMultiDrawInstances(q._multiDrawStarts,q._multiDrawCounts,q._multiDrawCount,q._multiDrawInstances);else if(gt.get("WEBGL_multi_draw"))dt.renderMultiDraw(q._multiDrawStarts,q._multiDrawCounts,q._multiDrawCount);else{const $e=q._multiDrawStarts,bt=q._multiDrawCounts,Tt=q._multiDrawCount,Nn=Le?oe.get(Le).bytesPerElement:1,ji=Ae.get(le).currentProgram.getUniforms();for(let nn=0;nn<Tt;nn++)ji.setValue(B,"_gl_DrawID",nn),dt.render($e[nn]/Nn,bt[nn])}else if(q.isInstancedMesh)dt.renderInstances(St,It,q.count);else if(re.isInstancedBufferGeometry){const $e=re._maxInstanceCount!==void 0?re._maxInstanceCount:1/0,bt=Math.min(re.instanceCount,$e);dt.renderInstances(St,It,bt)}else dt.render(St,It)};function at(C,X,re){C.transparent===!0&&C.side===Bi&&C.forceSinglePass===!1?(C.side=Ln,C.needsUpdate=!0,Wi(C,X,re),C.side=Mr,C.needsUpdate=!0,Wi(C,X,re),C.side=Bi):Wi(C,X,re)}this.compile=function(C,X,re=null){re===null&&(re=C),x=Be.get(re),x.init(X),P.push(x),re.traverseVisible(function(q){q.isLight&&q.layers.test(X.layers)&&(x.pushLight(q),q.castShadow&&x.pushShadow(q))}),C!==re&&C.traverseVisible(function(q){q.isLight&&q.layers.test(X.layers)&&(x.pushLight(q),q.castShadow&&x.pushShadow(q))}),x.setupLights();const le=new Set;return C.traverse(function(q){const Pe=q.material;if(Pe)if(Array.isArray(Pe))for(let ze=0;ze<Pe.length;ze++){const Xe=Pe[ze];at(Xe,re,q),le.add(Xe)}else at(Pe,re,q),le.add(Pe)}),P.pop(),x=null,le},this.compileAsync=function(C,X,re=null){const le=this.compile(C,X,re);return new Promise(q=>{function Pe(){if(le.forEach(function(ze){Ae.get(ze).currentProgram.isReady()&&le.delete(ze)}),le.size===0){q(C);return}setTimeout(Pe,10)}gt.get("KHR_parallel_shader_compile")!==null?Pe():setTimeout(Pe,10)})};let tt=null;function kt(C){tt&&tt(C)}function qt(){Qn.stop()}function En(){Qn.start()}const Qn=new s_;Qn.setAnimationLoop(kt),typeof self<"u"&&Qn.setContext(self),this.setAnimationLoop=function(C){tt=C,Ce.setAnimationLoop(C),C===null?Qn.stop():Qn.start()},Ce.addEventListener("sessionstart",qt),Ce.addEventListener("sessionend",En),this.render=function(C,X){if(X!==void 0&&X.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(L===!0)return;if(C.matrixWorldAutoUpdate===!0&&C.updateMatrixWorld(),X.parent===null&&X.matrixWorldAutoUpdate===!0&&X.updateMatrixWorld(),Ce.enabled===!0&&Ce.isPresenting===!0&&(Ce.cameraAutoUpdate===!0&&Ce.updateCamera(X),X=Ce.getCamera()),C.isScene===!0&&C.onBeforeRender(R,C,X,N),x=Be.get(C,P.length),x.init(X),P.push(x),Se.multiplyMatrices(X.projectionMatrix,X.matrixWorldInverse),ke.setFromProjectionMatrix(Se),de=this.localClippingEnabled,J=ct.init(this.clippingPlanes,de),S=Ne.get(C,_.length),S.init(),_.push(S),Ce.enabled===!0&&Ce.isPresenting===!0){const Pe=R.xr.getDepthSensingMesh();Pe!==null&&Mi(Pe,X,-1/0,R.sortObjects)}Mi(C,X,0,R.sortObjects),S.finish(),R.sortObjects===!0&&S.sort(G,he),Qe=Ce.enabled===!1||Ce.isPresenting===!1||Ce.hasDepthSensing()===!1,Qe&&Oe.addToRenderList(S,C),this.info.render.frame++,J===!0&&ct.beginShadows();const re=x.state.shadowsArray;Te.render(re,C,X),J===!0&&ct.endShadows(),this.info.autoReset===!0&&this.info.reset();const le=S.opaque,q=S.transmissive;if(x.setupLights(),X.isArrayCamera){const Pe=X.cameras;if(q.length>0)for(let ze=0,Xe=Pe.length;ze<Xe;ze++){const Le=Pe[ze];Gi(le,q,C,Le)}Qe&&Oe.render(C);for(let ze=0,Xe=Pe.length;ze<Xe;ze++){const Le=Pe[ze];oo(S,C,Le,Le.viewport)}}else q.length>0&&Gi(le,q,C,X),Qe&&Oe.render(C),oo(S,C,X);N!==null&&(Ue.updateMultisampleRenderTarget(N),Ue.updateRenderTargetMipmap(N)),C.isScene===!0&&C.onAfterRender(R,C,X),ft.resetDefaultState(),H=-1,b=null,P.pop(),P.length>0?(x=P[P.length-1],J===!0&&ct.setGlobalState(R.clippingPlanes,x.state.camera)):x=null,_.pop(),_.length>0?S=_[_.length-1]:S=null};function Mi(C,X,re,le){if(C.visible===!1)return;if(C.layers.test(X.layers)){if(C.isGroup)re=C.renderOrder;else if(C.isLOD)C.autoUpdate===!0&&C.update(X);else if(C.isLight)x.pushLight(C),C.castShadow&&x.pushShadow(C);else if(C.isSprite){if(!C.frustumCulled||ke.intersectsSprite(C)){le&&Ie.setFromMatrixPosition(C.matrixWorld).applyMatrix4(Se);const ze=Ee.update(C),Xe=C.material;Xe.visible&&S.push(C,ze,Xe,re,Ie.z,null)}}else if((C.isMesh||C.isLine||C.isPoints)&&(!C.frustumCulled||ke.intersectsObject(C))){const ze=Ee.update(C),Xe=C.material;if(le&&(C.boundingSphere!==void 0?(C.boundingSphere===null&&C.computeBoundingSphere(),Ie.copy(C.boundingSphere.center)):(ze.boundingSphere===null&&ze.computeBoundingSphere(),Ie.copy(ze.boundingSphere.center)),Ie.applyMatrix4(C.matrixWorld).applyMatrix4(Se)),Array.isArray(Xe)){const Le=ze.groups;for(let ot=0,st=Le.length;ot<st;ot++){const et=Le[ot],St=Xe[et.materialIndex];St&&St.visible&&S.push(C,ze,St,re,Ie.z,et)}}else Xe.visible&&S.push(C,ze,Xe,re,Ie.z,null)}}const Pe=C.children;for(let ze=0,Xe=Pe.length;ze<Xe;ze++)Mi(Pe[ze],X,re,le)}function oo(C,X,re,le){const q=C.opaque,Pe=C.transmissive,ze=C.transparent;x.setupLightsView(re),J===!0&&ct.setGlobalState(R.clippingPlanes,re),le&&ie.viewport(w.copy(le)),q.length>0&&Ei(q,X,re),Pe.length>0&&Ei(Pe,X,re),ze.length>0&&Ei(ze,X,re),ie.buffers.depth.setTest(!0),ie.buffers.depth.setMask(!0),ie.buffers.color.setMask(!0),ie.setPolygonOffset(!1)}function Gi(C,X,re,le){if((re.isScene===!0?re.overrideMaterial:null)!==null)return;x.state.transmissionRenderTarget[le.id]===void 0&&(x.state.transmissionRenderTarget[le.id]=new ts(1,1,{generateMipmaps:!0,type:gt.has("EXT_color_buffer_half_float")||gt.has("EXT_color_buffer_float")?ro:Vi,minFilter:Zr,samples:4,stencilBuffer:u,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:Rt.workingColorSpace}));const Pe=x.state.transmissionRenderTarget[le.id],ze=le.viewport||w;Pe.setSize(ze.z,ze.w);const Xe=R.getRenderTarget();R.setRenderTarget(Pe),R.getClearColor(V),Z=R.getClearAlpha(),Z<1&&R.setClearColor(16777215,.5),R.clear(),Qe&&Oe.render(re);const Le=R.toneMapping;R.toneMapping=yr;const ot=le.viewport;if(le.viewport!==void 0&&(le.viewport=void 0),x.setupLightsView(le),J===!0&&ct.setGlobalState(R.clippingPlanes,le),Ei(C,re,le),Ue.updateMultisampleRenderTarget(Pe),Ue.updateRenderTargetMipmap(Pe),gt.has("WEBGL_multisampled_render_to_texture")===!1){let st=!1;for(let et=0,St=X.length;et<St;et++){const Bt=X[et],It=Bt.object,tn=Bt.geometry,dt=Bt.material,$e=Bt.group;if(dt.side===Bi&&It.layers.test(le.layers)){const bt=dt.side;dt.side=Ln,dt.needsUpdate=!0,Tr(It,re,le,tn,dt,$e),dt.side=bt,dt.needsUpdate=!0,st=!0}}st===!0&&(Ue.updateMultisampleRenderTarget(Pe),Ue.updateRenderTargetMipmap(Pe))}R.setRenderTarget(Xe),R.setClearColor(V,Z),ot!==void 0&&(le.viewport=ot),R.toneMapping=Le}function Ei(C,X,re){const le=X.isScene===!0?X.overrideMaterial:null;for(let q=0,Pe=C.length;q<Pe;q++){const ze=C[q],Xe=ze.object,Le=ze.geometry,ot=le===null?ze.material:le,st=ze.group;Xe.layers.test(re.layers)&&Tr(Xe,X,re,Le,ot,st)}}function Tr(C,X,re,le,q,Pe){C.onBeforeRender(R,X,re,le,q,Pe),C.modelViewMatrix.multiplyMatrices(re.matrixWorldInverse,C.matrixWorld),C.normalMatrix.getNormalMatrix(C.modelViewMatrix),q.transparent===!0&&q.side===Bi&&q.forceSinglePass===!1?(q.side=Ln,q.needsUpdate=!0,R.renderBufferDirect(re,X,le,q,C,Pe),q.side=Mr,q.needsUpdate=!0,R.renderBufferDirect(re,X,le,q,C,Pe),q.side=Bi):R.renderBufferDirect(re,X,le,q,C,Pe),C.onAfterRender(R,X,re,le,q,Pe)}function Wi(C,X,re){X.isScene!==!0&&(X=Ge);const le=Ae.get(C),q=x.state.lights,Pe=x.state.shadowsArray,ze=q.state.version,Xe=ge.getParameters(C,q.state,Pe,X,re),Le=ge.getProgramCacheKey(Xe);let ot=le.programs;le.environment=C.isMeshStandardMaterial?X.environment:null,le.fog=X.fog,le.envMap=(C.isMeshStandardMaterial?A:F).get(C.envMap||le.environment),le.envMapRotation=le.environment!==null&&C.envMap===null?X.environmentRotation:C.envMapRotation,ot===void 0&&(C.addEventListener("dispose",Je),ot=new Map,le.programs=ot);let st=ot.get(Le);if(st!==void 0){if(le.currentProgram===st&&le.lightsStateVersion===ze)return uo(C,Xe),st}else Xe.uniforms=ge.getUniforms(C),C.onBeforeCompile(Xe,R),st=ge.acquireProgram(Xe,Le),ot.set(Le,st),le.uniforms=Xe.uniforms;const et=le.uniforms;return(!C.isShaderMaterial&&!C.isRawShaderMaterial||C.clipping===!0)&&(et.clippingPlanes=ct.uniform),uo(C,Xe),le.needsLights=co(C),le.lightsStateVersion=ze,le.needsLights&&(et.ambientLightColor.value=q.state.ambient,et.lightProbe.value=q.state.probe,et.directionalLights.value=q.state.directional,et.directionalLightShadows.value=q.state.directionalShadow,et.spotLights.value=q.state.spot,et.spotLightShadows.value=q.state.spotShadow,et.rectAreaLights.value=q.state.rectArea,et.ltc_1.value=q.state.rectAreaLTC1,et.ltc_2.value=q.state.rectAreaLTC2,et.pointLights.value=q.state.point,et.pointLightShadows.value=q.state.pointShadow,et.hemisphereLights.value=q.state.hemi,et.directionalShadowMap.value=q.state.directionalShadowMap,et.directionalShadowMatrix.value=q.state.directionalShadowMatrix,et.spotShadowMap.value=q.state.spotShadowMap,et.spotLightMatrix.value=q.state.spotLightMatrix,et.spotLightMap.value=q.state.spotLightMap,et.pointShadowMap.value=q.state.pointShadowMap,et.pointShadowMatrix.value=q.state.pointShadowMatrix),le.currentProgram=st,le.uniformsList=null,st}function lo(C){if(C.uniformsList===null){const X=C.currentProgram.getUniforms();C.uniformsList=Jl.seqWithValue(X.seq,C.uniforms)}return C.uniformsList}function uo(C,X){const re=Ae.get(C);re.outputColorSpace=X.outputColorSpace,re.batching=X.batching,re.batchingColor=X.batchingColor,re.instancing=X.instancing,re.instancingColor=X.instancingColor,re.instancingMorph=X.instancingMorph,re.skinning=X.skinning,re.morphTargets=X.morphTargets,re.morphNormals=X.morphNormals,re.morphColors=X.morphColors,re.morphTargetsCount=X.morphTargetsCount,re.numClippingPlanes=X.numClippingPlanes,re.numIntersection=X.numClipIntersection,re.vertexAlphas=X.vertexAlphas,re.vertexTangents=X.vertexTangents,re.toneMapping=X.toneMapping}function _u(C,X,re,le,q){X.isScene!==!0&&(X=Ge),Ue.resetTextureUnits();const Pe=X.fog,ze=le.isMeshStandardMaterial?X.environment:null,Xe=N===null?R.outputColorSpace:N.isXRRenderTarget===!0?N.texture.colorSpace:wr,Le=(le.isMeshStandardMaterial?A:F).get(le.envMap||ze),ot=le.vertexColors===!0&&!!re.attributes.color&&re.attributes.color.itemSize===4,st=!!re.attributes.tangent&&(!!le.normalMap||le.anisotropy>0),et=!!re.morphAttributes.position,St=!!re.morphAttributes.normal,Bt=!!re.morphAttributes.color;let It=yr;le.toneMapped&&(N===null||N.isXRRenderTarget===!0)&&(It=R.toneMapping);const tn=re.morphAttributes.position||re.morphAttributes.normal||re.morphAttributes.color,dt=tn!==void 0?tn.length:0,$e=Ae.get(le),bt=x.state.lights;if(J===!0&&(de===!0||C!==b)){const wn=C===b&&le.id===H;ct.setState(le,C,wn)}let Tt=!1;le.version===$e.__version?($e.needsLights&&$e.lightsStateVersion!==bt.state.version||$e.outputColorSpace!==Xe||q.isBatchedMesh&&$e.batching===!1||!q.isBatchedMesh&&$e.batching===!0||q.isBatchedMesh&&$e.batchingColor===!0&&q.colorTexture===null||q.isBatchedMesh&&$e.batchingColor===!1&&q.colorTexture!==null||q.isInstancedMesh&&$e.instancing===!1||!q.isInstancedMesh&&$e.instancing===!0||q.isSkinnedMesh&&$e.skinning===!1||!q.isSkinnedMesh&&$e.skinning===!0||q.isInstancedMesh&&$e.instancingColor===!0&&q.instanceColor===null||q.isInstancedMesh&&$e.instancingColor===!1&&q.instanceColor!==null||q.isInstancedMesh&&$e.instancingMorph===!0&&q.morphTexture===null||q.isInstancedMesh&&$e.instancingMorph===!1&&q.morphTexture!==null||$e.envMap!==Le||le.fog===!0&&$e.fog!==Pe||$e.numClippingPlanes!==void 0&&($e.numClippingPlanes!==ct.numPlanes||$e.numIntersection!==ct.numIntersection)||$e.vertexAlphas!==ot||$e.vertexTangents!==st||$e.morphTargets!==et||$e.morphNormals!==St||$e.morphColors!==Bt||$e.toneMapping!==It||$e.morphTargetsCount!==dt)&&(Tt=!0):(Tt=!0,$e.__version=le.version);let Nn=$e.currentProgram;Tt===!0&&(Nn=Wi(le,X,q));let ji=!1,nn=!1,Xi=!1;const Nt=Nn.getUniforms(),Dn=$e.uniforms;if(ie.useProgram(Nn.program)&&(ji=!0,nn=!0,Xi=!0),le.id!==H&&(H=le.id,nn=!0),ji||b!==C){Nt.setValue(B,"projectionMatrix",C.projectionMatrix),Nt.setValue(B,"viewMatrix",C.matrixWorldInverse);const wn=Nt.map.cameraPosition;wn!==void 0&&wn.setValue(B,Me.setFromMatrixPosition(C.matrixWorld)),ne.logarithmicDepthBuffer&&Nt.setValue(B,"logDepthBufFC",2/(Math.log(C.far+1)/Math.LN2)),(le.isMeshPhongMaterial||le.isMeshToonMaterial||le.isMeshLambertMaterial||le.isMeshBasicMaterial||le.isMeshStandardMaterial||le.isShaderMaterial)&&Nt.setValue(B,"isOrthographic",C.isOrthographicCamera===!0),b!==C&&(b=C,nn=!0,Xi=!0)}if(q.isSkinnedMesh){Nt.setOptional(B,q,"bindMatrix"),Nt.setOptional(B,q,"bindMatrixInverse");const wn=q.skeleton;wn&&(wn.boneTexture===null&&wn.computeBoneTexture(),Nt.setValue(B,"boneTexture",wn.boneTexture,Ue))}q.isBatchedMesh&&(Nt.setOptional(B,q,"batchingTexture"),Nt.setValue(B,"batchingTexture",q._matricesTexture,Ue),Nt.setOptional(B,q,"batchingIdTexture"),Nt.setValue(B,"batchingIdTexture",q._indirectTexture,Ue),Nt.setOptional(B,q,"batchingColorTexture"),q._colorsTexture!==null&&Nt.setValue(B,"batchingColorTexture",q._colorsTexture,Ue));const da=re.morphAttributes;if((da.position!==void 0||da.normal!==void 0||da.color!==void 0)&&_t.update(q,re,Nn),(nn||$e.receiveShadow!==q.receiveShadow)&&($e.receiveShadow=q.receiveShadow,Nt.setValue(B,"receiveShadow",q.receiveShadow)),le.isMeshGouraudMaterial&&le.envMap!==null&&(Dn.envMap.value=Le,Dn.flipEnvMap.value=Le.isCubeTexture&&Le.isRenderTargetTexture===!1?-1:1),le.isMeshStandardMaterial&&le.envMap===null&&X.environment!==null&&(Dn.envMapIntensity.value=X.environmentIntensity),nn&&(Nt.setValue(B,"toneMappingExposure",R.toneMappingExposure),$e.needsLights&&wi(Dn,Xi),Pe&&le.fog===!0&&Ze.refreshFogUniforms(Dn,Pe),Ze.refreshMaterialUniforms(Dn,le,se,ue,x.state.transmissionRenderTarget[C.id]),Jl.upload(B,lo($e),Dn,Ue)),le.isShaderMaterial&&le.uniformsNeedUpdate===!0&&(Jl.upload(B,lo($e),Dn,Ue),le.uniformsNeedUpdate=!1),le.isSpriteMaterial&&Nt.setValue(B,"center",q.center),Nt.setValue(B,"modelViewMatrix",q.modelViewMatrix),Nt.setValue(B,"normalMatrix",q.normalMatrix),Nt.setValue(B,"modelMatrix",q.matrixWorld),le.isShaderMaterial||le.isRawShaderMaterial){const wn=le.uniformsGroups;for(let Ar=0,fo=wn.length;Ar<fo;Ar++){const rs=wn[Ar];Ct.update(rs,Nn),Ct.bind(rs,Nn)}}return Nn}function wi(C,X){C.ambientLightColor.needsUpdate=X,C.lightProbe.needsUpdate=X,C.directionalLights.needsUpdate=X,C.directionalLightShadows.needsUpdate=X,C.pointLights.needsUpdate=X,C.pointLightShadows.needsUpdate=X,C.spotLights.needsUpdate=X,C.spotLightShadows.needsUpdate=X,C.rectAreaLights.needsUpdate=X,C.hemisphereLights.needsUpdate=X}function co(C){return C.isMeshLambertMaterial||C.isMeshToonMaterial||C.isMeshPhongMaterial||C.isMeshStandardMaterial||C.isShadowMaterial||C.isShaderMaterial&&C.lights===!0}this.getActiveCubeFace=function(){return Y},this.getActiveMipmapLevel=function(){return U},this.getRenderTarget=function(){return N},this.setRenderTargetTextures=function(C,X,re){Ae.get(C.texture).__webglTexture=X,Ae.get(C.depthTexture).__webglTexture=re;const le=Ae.get(C);le.__hasExternalTextures=!0,le.__autoAllocateDepthBuffer=re===void 0,le.__autoAllocateDepthBuffer||gt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),le.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(C,X){const re=Ae.get(C);re.__webglFramebuffer=X,re.__useDefaultFramebuffer=X===void 0},this.setRenderTarget=function(C,X=0,re=0){N=C,Y=X,U=re;let le=!0,q=null,Pe=!1,ze=!1;if(C){const Le=Ae.get(C);Le.__useDefaultFramebuffer!==void 0?(ie.bindFramebuffer(B.FRAMEBUFFER,null),le=!1):Le.__webglFramebuffer===void 0?Ue.setupRenderTarget(C):Le.__hasExternalTextures&&Ue.rebindTextures(C,Ae.get(C.texture).__webglTexture,Ae.get(C.depthTexture).__webglTexture);const ot=C.texture;(ot.isData3DTexture||ot.isDataArrayTexture||ot.isCompressedArrayTexture)&&(ze=!0);const st=Ae.get(C).__webglFramebuffer;C.isWebGLCubeRenderTarget?(Array.isArray(st[X])?q=st[X][re]:q=st[X],Pe=!0):C.samples>0&&Ue.useMultisampledRTT(C)===!1?q=Ae.get(C).__webglMultisampledFramebuffer:Array.isArray(st)?q=st[re]:q=st,w.copy(C.viewport),I.copy(C.scissor),K=C.scissorTest}else w.copy(fe).multiplyScalar(se).floor(),I.copy(k).multiplyScalar(se).floor(),K=ce;if(ie.bindFramebuffer(B.FRAMEBUFFER,q)&&le&&ie.drawBuffers(C,q),ie.viewport(w),ie.scissor(I),ie.setScissorTest(K),Pe){const Le=Ae.get(C.texture);B.framebufferTexture2D(B.FRAMEBUFFER,B.COLOR_ATTACHMENT0,B.TEXTURE_CUBE_MAP_POSITIVE_X+X,Le.__webglTexture,re)}else if(ze){const Le=Ae.get(C.texture),ot=X||0;B.framebufferTextureLayer(B.FRAMEBUFFER,B.COLOR_ATTACHMENT0,Le.__webglTexture,re||0,ot)}H=-1},this.readRenderTargetPixels=function(C,X,re,le,q,Pe,ze){if(!(C&&C.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let Xe=Ae.get(C).__webglFramebuffer;if(C.isWebGLCubeRenderTarget&&ze!==void 0&&(Xe=Xe[ze]),Xe){ie.bindFramebuffer(B.FRAMEBUFFER,Xe);try{const Le=C.texture,ot=Le.format,st=Le.type;if(!ne.textureFormatReadable(ot)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!ne.textureTypeReadable(st)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}X>=0&&X<=C.width-le&&re>=0&&re<=C.height-q&&B.readPixels(X,re,le,q,lt.convert(ot),lt.convert(st),Pe)}finally{const Le=N!==null?Ae.get(N).__webglFramebuffer:null;ie.bindFramebuffer(B.FRAMEBUFFER,Le)}}},this.readRenderTargetPixelsAsync=async function(C,X,re,le,q,Pe,ze){if(!(C&&C.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let Xe=Ae.get(C).__webglFramebuffer;if(C.isWebGLCubeRenderTarget&&ze!==void 0&&(Xe=Xe[ze]),Xe){ie.bindFramebuffer(B.FRAMEBUFFER,Xe);try{const Le=C.texture,ot=Le.format,st=Le.type;if(!ne.textureFormatReadable(ot))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!ne.textureTypeReadable(st))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(X>=0&&X<=C.width-le&&re>=0&&re<=C.height-q){const et=B.createBuffer();B.bindBuffer(B.PIXEL_PACK_BUFFER,et),B.bufferData(B.PIXEL_PACK_BUFFER,Pe.byteLength,B.STREAM_READ),B.readPixels(X,re,le,q,lt.convert(ot),lt.convert(st),0),B.flush();const St=B.fenceSync(B.SYNC_GPU_COMMANDS_COMPLETE,0);await sx(B,St,4);try{B.bindBuffer(B.PIXEL_PACK_BUFFER,et),B.getBufferSubData(B.PIXEL_PACK_BUFFER,0,Pe)}finally{B.deleteBuffer(et),B.deleteSync(St)}return Pe}}finally{const Le=N!==null?Ae.get(N).__webglFramebuffer:null;ie.bindFramebuffer(B.FRAMEBUFFER,Le)}}},this.copyFramebufferToTexture=function(C,X=null,re=0){C.isTexture!==!0&&(ta("WebGLRenderer: copyFramebufferToTexture function signature has changed."),X=arguments[0]||null,C=arguments[1]);const le=Math.pow(2,-re),q=Math.floor(C.image.width*le),Pe=Math.floor(C.image.height*le),ze=X!==null?X.x:0,Xe=X!==null?X.y:0;Ue.setTexture2D(C,0),B.copyTexSubImage2D(B.TEXTURE_2D,re,0,0,ze,Xe,q,Pe),ie.unbindTexture()},this.copyTextureToTexture=function(C,X,re=null,le=null,q=0){C.isTexture!==!0&&(ta("WebGLRenderer: copyTextureToTexture function signature has changed."),le=arguments[0]||null,C=arguments[1],X=arguments[2],q=arguments[3]||0,re=null);let Pe,ze,Xe,Le,ot,st;re!==null?(Pe=re.max.x-re.min.x,ze=re.max.y-re.min.y,Xe=re.min.x,Le=re.min.y):(Pe=C.image.width,ze=C.image.height,Xe=0,Le=0),le!==null?(ot=le.x,st=le.y):(ot=0,st=0);const et=lt.convert(X.format),St=lt.convert(X.type);Ue.setTexture2D(X,0),B.pixelStorei(B.UNPACK_FLIP_Y_WEBGL,X.flipY),B.pixelStorei(B.UNPACK_PREMULTIPLY_ALPHA_WEBGL,X.premultiplyAlpha),B.pixelStorei(B.UNPACK_ALIGNMENT,X.unpackAlignment);const Bt=B.getParameter(B.UNPACK_ROW_LENGTH),It=B.getParameter(B.UNPACK_IMAGE_HEIGHT),tn=B.getParameter(B.UNPACK_SKIP_PIXELS),dt=B.getParameter(B.UNPACK_SKIP_ROWS),$e=B.getParameter(B.UNPACK_SKIP_IMAGES),bt=C.isCompressedTexture?C.mipmaps[q]:C.image;B.pixelStorei(B.UNPACK_ROW_LENGTH,bt.width),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,bt.height),B.pixelStorei(B.UNPACK_SKIP_PIXELS,Xe),B.pixelStorei(B.UNPACK_SKIP_ROWS,Le),C.isDataTexture?B.texSubImage2D(B.TEXTURE_2D,q,ot,st,Pe,ze,et,St,bt.data):C.isCompressedTexture?B.compressedTexSubImage2D(B.TEXTURE_2D,q,ot,st,bt.width,bt.height,et,bt.data):B.texSubImage2D(B.TEXTURE_2D,q,ot,st,Pe,ze,et,St,bt),B.pixelStorei(B.UNPACK_ROW_LENGTH,Bt),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,It),B.pixelStorei(B.UNPACK_SKIP_PIXELS,tn),B.pixelStorei(B.UNPACK_SKIP_ROWS,dt),B.pixelStorei(B.UNPACK_SKIP_IMAGES,$e),q===0&&X.generateMipmaps&&B.generateMipmap(B.TEXTURE_2D),ie.unbindTexture()},this.copyTextureToTexture3D=function(C,X,re=null,le=null,q=0){C.isTexture!==!0&&(ta("WebGLRenderer: copyTextureToTexture3D function signature has changed."),re=arguments[0]||null,le=arguments[1]||null,C=arguments[2],X=arguments[3],q=arguments[4]||0);let Pe,ze,Xe,Le,ot,st,et,St,Bt;const It=C.isCompressedTexture?C.mipmaps[q]:C.image;re!==null?(Pe=re.max.x-re.min.x,ze=re.max.y-re.min.y,Xe=re.max.z-re.min.z,Le=re.min.x,ot=re.min.y,st=re.min.z):(Pe=It.width,ze=It.height,Xe=It.depth,Le=0,ot=0,st=0),le!==null?(et=le.x,St=le.y,Bt=le.z):(et=0,St=0,Bt=0);const tn=lt.convert(X.format),dt=lt.convert(X.type);let $e;if(X.isData3DTexture)Ue.setTexture3D(X,0),$e=B.TEXTURE_3D;else if(X.isDataArrayTexture||X.isCompressedArrayTexture)Ue.setTexture2DArray(X,0),$e=B.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}B.pixelStorei(B.UNPACK_FLIP_Y_WEBGL,X.flipY),B.pixelStorei(B.UNPACK_PREMULTIPLY_ALPHA_WEBGL,X.premultiplyAlpha),B.pixelStorei(B.UNPACK_ALIGNMENT,X.unpackAlignment);const bt=B.getParameter(B.UNPACK_ROW_LENGTH),Tt=B.getParameter(B.UNPACK_IMAGE_HEIGHT),Nn=B.getParameter(B.UNPACK_SKIP_PIXELS),ji=B.getParameter(B.UNPACK_SKIP_ROWS),nn=B.getParameter(B.UNPACK_SKIP_IMAGES);B.pixelStorei(B.UNPACK_ROW_LENGTH,It.width),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,It.height),B.pixelStorei(B.UNPACK_SKIP_PIXELS,Le),B.pixelStorei(B.UNPACK_SKIP_ROWS,ot),B.pixelStorei(B.UNPACK_SKIP_IMAGES,st),C.isDataTexture||C.isData3DTexture?B.texSubImage3D($e,q,et,St,Bt,Pe,ze,Xe,tn,dt,It.data):X.isCompressedArrayTexture?B.compressedTexSubImage3D($e,q,et,St,Bt,Pe,ze,Xe,tn,It.data):B.texSubImage3D($e,q,et,St,Bt,Pe,ze,Xe,tn,dt,It),B.pixelStorei(B.UNPACK_ROW_LENGTH,bt),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,Tt),B.pixelStorei(B.UNPACK_SKIP_PIXELS,Nn),B.pixelStorei(B.UNPACK_SKIP_ROWS,ji),B.pixelStorei(B.UNPACK_SKIP_IMAGES,nn),q===0&&X.generateMipmaps&&B.generateMipmap($e),ie.unbindTexture()},this.initRenderTarget=function(C){Ae.get(C).__webglFramebuffer===void 0&&Ue.setupRenderTarget(C)},this.initTexture=function(C){C.isCubeTexture?Ue.setTextureCube(C,0):C.isData3DTexture?Ue.setTexture3D(C,0):C.isDataArrayTexture||C.isCompressedArrayTexture?Ue.setTexture2DArray(C,0):Ue.setTexture2D(C,0),ie.unbindTexture()},this.resetState=function(){Y=0,U=0,N=null,ie.reset(),ft.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return Hi}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;const t=this.getContext();t.drawingBufferColorSpace=e===bd?"display-p3":"srgb",t.unpackColorSpace=Rt.workingColorSpace===fu?"display-p3":"srgb"}}class f_ extends Qt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Si,this.environmentIntensity=1,this.environmentRotation=new Si,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){const t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}}class mw{constructor(e,t){this.isInterleavedBuffer=!0,this.array=e,this.stride=t,this.count=e!==void 0?e.length/t:0,this.usage=vd,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.version=0,this.uuid=Sr()}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return ta("THREE.InterleavedBuffer: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.array=new e.array.constructor(e.array),this.count=e.count,this.stride=e.stride,this.usage=e.usage,this}copyAt(e,t,r){e*=this.stride,r*=t.stride;for(let o=0,u=this.stride;o<u;o++)this.array[e+o]=t.array[r+o];return this}set(e,t=0){return this.array.set(e,t),this}clone(e){e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Sr()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=this.array.slice(0).buffer);const t=new this.array.constructor(e.arrayBuffers[this.array.buffer._uuid]),r=new this.constructor(t,this.stride);return r.setUsage(this.usage),r}onUpload(e){return this.onUploadCallback=e,this}toJSON(e){return e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Sr()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=Array.from(new Uint32Array(this.array.buffer))),{uuid:this.uuid,buffer:this.array.buffer._uuid,type:this.array.constructor.name,stride:this.stride}}}const xn=new Q;class ou{constructor(e,t,r,o=!1){this.isInterleavedBufferAttribute=!0,this.name="",this.data=e,this.itemSize=t,this.offset=r,this.normalized=o}get count(){return this.data.count}get array(){return this.data.array}set needsUpdate(e){this.data.needsUpdate=e}applyMatrix4(e){for(let t=0,r=this.data.count;t<r;t++)xn.fromBufferAttribute(this,t),xn.applyMatrix4(e),this.setXYZ(t,xn.x,xn.y,xn.z);return this}applyNormalMatrix(e){for(let t=0,r=this.count;t<r;t++)xn.fromBufferAttribute(this,t),xn.applyNormalMatrix(e),this.setXYZ(t,xn.x,xn.y,xn.z);return this}transformDirection(e){for(let t=0,r=this.count;t<r;t++)xn.fromBufferAttribute(this,t),xn.transformDirection(e),this.setXYZ(t,xn.x,xn.y,xn.z);return this}getComponent(e,t){let r=this.array[e*this.data.stride+this.offset+t];return this.normalized&&(r=xi(r,this.array)),r}setComponent(e,t,r){return this.normalized&&(r=Lt(r,this.array)),this.data.array[e*this.data.stride+this.offset+t]=r,this}setX(e,t){return this.normalized&&(t=Lt(t,this.array)),this.data.array[e*this.data.stride+this.offset]=t,this}setY(e,t){return this.normalized&&(t=Lt(t,this.array)),this.data.array[e*this.data.stride+this.offset+1]=t,this}setZ(e,t){return this.normalized&&(t=Lt(t,this.array)),this.data.array[e*this.data.stride+this.offset+2]=t,this}setW(e,t){return this.normalized&&(t=Lt(t,this.array)),this.data.array[e*this.data.stride+this.offset+3]=t,this}getX(e){let t=this.data.array[e*this.data.stride+this.offset];return this.normalized&&(t=xi(t,this.array)),t}getY(e){let t=this.data.array[e*this.data.stride+this.offset+1];return this.normalized&&(t=xi(t,this.array)),t}getZ(e){let t=this.data.array[e*this.data.stride+this.offset+2];return this.normalized&&(t=xi(t,this.array)),t}getW(e){let t=this.data.array[e*this.data.stride+this.offset+3];return this.normalized&&(t=xi(t,this.array)),t}setXY(e,t,r){return e=e*this.data.stride+this.offset,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this}setXYZ(e,t,r,o){return e=e*this.data.stride+this.offset,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array),o=Lt(o,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this.data.array[e+2]=o,this}setXYZW(e,t,r,o,u){return e=e*this.data.stride+this.offset,this.normalized&&(t=Lt(t,this.array),r=Lt(r,this.array),o=Lt(o,this.array),u=Lt(u,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this.data.array[e+2]=o,this.data.array[e+3]=u,this}clone(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.clone(): Cloning an interleaved buffer attribute will de-interleave buffer data.");const t=[];for(let r=0;r<this.count;r++){const o=r*this.data.stride+this.offset;for(let u=0;u<this.itemSize;u++)t.push(this.data.array[o+u])}return new hi(new this.array.constructor(t),this.itemSize,this.normalized)}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.clone(e)),new ou(e.interleavedBuffers[this.data.uuid],this.itemSize,this.offset,this.normalized)}toJSON(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.toJSON(): Serializing an interleaved buffer attribute will de-interleave buffer data.");const t=[];for(let r=0;r<this.count;r++){const o=r*this.data.stride+this.offset;for(let u=0;u<this.itemSize;u++)t.push(this.data.array[o+u])}return{itemSize:this.itemSize,type:this.array.constructor.name,array:t,normalized:this.normalized}}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.toJSON(e)),{isInterleavedBufferAttribute:!0,itemSize:this.itemSize,data:this.data.uuid,offset:this.offset,normalized:this.normalized}}}class d_ extends is{constructor(e){super(),this.isSpriteMaterial=!0,this.type="SpriteMaterial",this.color=new yt(16777215),this.map=null,this.alphaMap=null,this.rotation=0,this.sizeAttenuation=!0,this.transparent=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.rotation=e.rotation,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}}let Ws;const Za=new Q,js=new Q,Xs=new Q,Ys=new ht,Qa=new ht,h_=new Gt,Fl=new Q,Ja=new Q,Ol=new Q,hg=new ht,bf=new ht,pg=new ht;class gw extends Qt{constructor(e=new d_){if(super(),this.isSprite=!0,this.type="Sprite",Ws===void 0){Ws=new Hn;const t=new Float32Array([-.5,-.5,0,0,0,.5,-.5,0,1,0,.5,.5,0,1,1,-.5,.5,0,0,1]),r=new mw(t,5);Ws.setIndex([0,1,2,0,2,3]),Ws.setAttribute("position",new ou(r,3,0,!1)),Ws.setAttribute("uv",new ou(r,2,3,!1))}this.geometry=Ws,this.material=e,this.center=new ht(.5,.5)}raycast(e,t){e.camera===null&&console.error('THREE.Sprite: "Raycaster.camera" needs to be set in order to raycast against sprites.'),js.setFromMatrixScale(this.matrixWorld),h_.copy(e.camera.matrixWorld),this.modelViewMatrix.multiplyMatrices(e.camera.matrixWorldInverse,this.matrixWorld),Xs.setFromMatrixPosition(this.modelViewMatrix),e.camera.isPerspectiveCamera&&this.material.sizeAttenuation===!1&&js.multiplyScalar(-Xs.z);const r=this.material.rotation;let o,u;r!==0&&(u=Math.cos(r),o=Math.sin(r));const c=this.center;kl(Fl.set(-.5,-.5,0),Xs,c,js,o,u),kl(Ja.set(.5,-.5,0),Xs,c,js,o,u),kl(Ol.set(.5,.5,0),Xs,c,js,o,u),hg.set(0,0),bf.set(1,0),pg.set(1,1);let f=e.ray.intersectTriangle(Fl,Ja,Ol,!1,Za);if(f===null&&(kl(Ja.set(-.5,.5,0),Xs,c,js,o,u),bf.set(0,1),f=e.ray.intersectTriangle(Fl,Ol,Ja,!1,Za),f===null))return;const h=e.ray.origin.distanceTo(Za);h<e.near||h>e.far||t.push({distance:h,point:Za.clone(),uv:fi.getInterpolation(Za,Fl,Ja,Ol,hg,bf,pg,new ht),face:null,object:this})}copy(e,t){return super.copy(e,t),e.center!==void 0&&this.center.copy(e.center),this.material=e.material,this}}function kl(s,e,t,r,o,u){Ys.subVectors(s,t).addScalar(.5).multiply(r),o!==void 0?(Qa.x=u*Ys.x-o*Ys.y,Qa.y=o*Ys.x+u*Ys.y):Qa.copy(Ys),s.copy(e),s.x+=Qa.x,s.y+=Qa.y,s.applyMatrix4(h_)}class mu extends is{constructor(e){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new yt(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.linewidth=e.linewidth,this.linecap=e.linecap,this.linejoin=e.linejoin,this.fog=e.fog,this}}const lu=new Q,uu=new Q,mg=new Gt,eo=new Zg,Bl=new du,Pf=new Q,gg=new Q;class p_ extends Qt{constructor(e=new Hn,t=new mu){super(),this.isLine=!0,this.type="Line",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,r=[0];for(let o=1,u=t.count;o<u;o++)lu.fromBufferAttribute(t,o-1),uu.fromBufferAttribute(t,o),r[o]=r[o-1],r[o]+=lu.distanceTo(uu);e.setAttribute("lineDistance",new gn(r,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(e,t){const r=this.geometry,o=this.matrixWorld,u=e.params.Line.threshold,c=r.drawRange;if(r.boundingSphere===null&&r.computeBoundingSphere(),Bl.copy(r.boundingSphere),Bl.applyMatrix4(o),Bl.radius+=u,e.ray.intersectsSphere(Bl)===!1)return;mg.copy(o).invert(),eo.copy(e.ray).applyMatrix4(mg);const f=u/((this.scale.x+this.scale.y+this.scale.z)/3),h=f*f,m=this.isLineSegments?2:1,g=r.index,v=r.attributes.position;if(g!==null){const M=Math.max(0,c.start),T=Math.min(g.count,c.start+c.count);for(let S=M,x=T-1;S<x;S+=m){const _=g.getX(S),P=g.getX(S+1),R=zl(this,e,eo,h,_,P);R&&t.push(R)}if(this.isLineLoop){const S=g.getX(T-1),x=g.getX(M),_=zl(this,e,eo,h,S,x);_&&t.push(_)}}else{const M=Math.max(0,c.start),T=Math.min(v.count,c.start+c.count);for(let S=M,x=T-1;S<x;S+=m){const _=zl(this,e,eo,h,S,S+1);_&&t.push(_)}if(this.isLineLoop){const S=zl(this,e,eo,h,T-1,M);S&&t.push(S)}}}updateMorphTargets(){const t=this.geometry.morphAttributes,r=Object.keys(t);if(r.length>0){const o=t[r[0]];if(o!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let u=0,c=o.length;u<c;u++){const f=o[u].name||String(u);this.morphTargetInfluences.push(0),this.morphTargetDictionary[f]=u}}}}}function zl(s,e,t,r,o,u){const c=s.geometry.attributes.position;if(lu.fromBufferAttribute(c,o),uu.fromBufferAttribute(c,u),t.distanceSqToSegment(lu,uu,Pf,gg)>r)return;Pf.applyMatrix4(s.matrixWorld);const h=e.ray.origin.distanceTo(Pf);if(!(h<e.near||h>e.far))return{distance:h,point:gg.clone().applyMatrix4(s.matrixWorld),index:o,face:null,faceIndex:null,object:s}}const _g=new Q,vg=new Q;class m_ extends p_{constructor(e,t){super(e,t),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,r=[];for(let o=0,u=t.count;o<u;o+=2)_g.fromBufferAttribute(t,o),vg.fromBufferAttribute(t,o+1),r[o]=o===0?0:r[o-1],r[o+1]=r[o]+_g.distanceTo(vg);e.setAttribute("lineDistance",new gn(r,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class _w extends Mn{constructor(e,t,r,o,u,c,f,h,m){super(e,t,r,o,u,c,f,h,m),this.isCanvasTexture=!0,this.needsUpdate=!0}}class Dd extends Hn{constructor(e=1,t=1,r=1,o=32,u=1,c=!1,f=0,h=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:e,radiusBottom:t,height:r,radialSegments:o,heightSegments:u,openEnded:c,thetaStart:f,thetaLength:h};const m=this;o=Math.floor(o),u=Math.floor(u);const g=[],y=[],v=[],M=[];let T=0;const S=[],x=r/2;let _=0;P(),c===!1&&(e>0&&R(!0),t>0&&R(!1)),this.setIndex(g),this.setAttribute("position",new gn(y,3)),this.setAttribute("normal",new gn(v,3)),this.setAttribute("uv",new gn(M,2));function P(){const L=new Q,Y=new Q;let U=0;const N=(t-e)/r;for(let H=0;H<=u;H++){const b=[],w=H/u,I=w*(t-e)+e;for(let K=0;K<=o;K++){const V=K/o,Z=V*h+f,te=Math.sin(Z),ue=Math.cos(Z);Y.x=I*te,Y.y=-w*r+x,Y.z=I*ue,y.push(Y.x,Y.y,Y.z),L.set(te,N,ue).normalize(),v.push(L.x,L.y,L.z),M.push(V,1-w),b.push(T++)}S.push(b)}for(let H=0;H<o;H++)for(let b=0;b<u;b++){const w=S[b][H],I=S[b+1][H],K=S[b+1][H+1],V=S[b][H+1];g.push(w,I,V),g.push(I,K,V),U+=6}m.addGroup(_,U,0),_+=U}function R(L){const Y=T,U=new ht,N=new Q;let H=0;const b=L===!0?e:t,w=L===!0?1:-1;for(let K=1;K<=o;K++)y.push(0,x*w,0),v.push(0,w,0),M.push(.5,.5),T++;const I=T;for(let K=0;K<=o;K++){const Z=K/o*h+f,te=Math.cos(Z),ue=Math.sin(Z);N.x=b*ue,N.y=x*w,N.z=b*te,y.push(N.x,N.y,N.z),v.push(0,w,0),U.x=te*.5+.5,U.y=ue*.5*w+.5,M.push(U.x,U.y),T++}for(let K=0;K<o;K++){const V=Y+K,Z=I+K;L===!0?g.push(Z,Z+1,V):g.push(Z+1,Z,V),H+=3}m.addGroup(_,H,L===!0?1:2),_+=H}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new Dd(e.radiusTop,e.radiusBottom,e.height,e.radialSegments,e.heightSegments,e.openEnded,e.thetaStart,e.thetaLength)}}class gu extends Dd{constructor(e=1,t=1,r=32,o=1,u=!1,c=0,f=Math.PI*2){super(0,e,t,r,o,u,c,f),this.type="ConeGeometry",this.parameters={radius:e,height:t,radialSegments:r,heightSegments:o,openEnded:u,thetaStart:c,thetaLength:f}}static fromJSON(e){return new gu(e.radius,e.height,e.radialSegments,e.heightSegments,e.openEnded,e.thetaStart,e.thetaLength)}}class Hl extends is{constructor(e){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new yt(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new yt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Xg,this.normalScale=new ht(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Si,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}}class g_ extends Qt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new yt(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){const t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(t.object.target=this.target.uuid),t}}class vw extends g_{constructor(e,t,r){super(e,r),this.isHemisphereLight=!0,this.type="HemisphereLight",this.position.copy(Qt.DEFAULT_UP),this.updateMatrix(),this.groundColor=new yt(t)}copy(e,t){return super.copy(e,t),this.groundColor.copy(e.groundColor),this}}const Lf=new Gt,xg=new Q,yg=new Q;class xw{constructor(e){this.camera=e,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new ht(512,512),this.map=null,this.mapPass=null,this.matrix=new Gt,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new Ld,this._frameExtents=new ht(1,1),this._viewportCount=1,this._viewports=[new Zt(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){const t=this.camera,r=this.matrix;xg.setFromMatrixPosition(e.matrixWorld),t.position.copy(xg),yg.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(yg),t.updateMatrixWorld(),Lf.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Lf),r.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),r.multiply(Lf)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.intensity=e.intensity,this.bias=e.bias,this.radius=e.radius,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const e={};return this.intensity!==1&&(e.intensity=this.intensity),this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}}class yw extends xw{constructor(){super(new Ks(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class Sw extends g_{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Qt.DEFAULT_UP),this.updateMatrix(),this.target=new Qt,this.shadow=new yw}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}}class Mw{constructor(e=1,t=0,r=0){return this.radius=e,this.phi=t,this.theta=r,this}set(e,t,r){return this.radius=e,this.phi=t,this.theta=r,this}copy(e){return this.radius=e.radius,this.phi=e.phi,this.theta=e.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(e){return this.setFromCartesianCoords(e.x,e.y,e.z)}setFromCartesianCoords(e,t,r){return this.radius=Math.sqrt(e*e+t*t+r*r),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(e,r),this.phi=Math.acos(Sn(t/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}class Ew extends m_{constructor(e=10,t=10,r=4473924,o=8947848){r=new yt(r),o=new yt(o);const u=t/2,c=e/t,f=e/2,h=[],m=[];for(let v=0,M=0,T=-f;v<=t;v++,T+=c){h.push(-f,0,T,f,0,T),h.push(T,0,-f,T,0,f);const S=v===u?r:o;S.toArray(m,M),M+=3,S.toArray(m,M),M+=3,S.toArray(m,M),M+=3,S.toArray(m,M),M+=3}const g=new Hn;g.setAttribute("position",new gn(h,3)),g.setAttribute("color",new gn(m,3));const y=new mu({vertexColors:!0,toneMapped:!1});super(g,y),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}class ww extends m_{constructor(e=1){const t=[0,0,0,e,0,0,0,0,0,0,e,0,0,0,0,0,0,e],r=[1,0,0,1,.6,0,0,1,0,.6,1,0,0,0,1,0,.6,1],o=new Hn;o.setAttribute("position",new gn(t,3)),o.setAttribute("color",new gn(r,3));const u=new mu({vertexColors:!0,toneMapped:!1});super(o,u),this.type="AxesHelper"}setColors(e,t,r){const o=new yt,u=this.geometry.attributes.color.array;return o.set(e),o.toArray(u,0),o.toArray(u,3),o.set(t),o.toArray(u,6),o.toArray(u,9),o.set(r),o.toArray(u,12),o.toArray(u,15),this.geometry.attributes.color.needsUpdate=!0,this}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:Md}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=Md);function Zn(s,e=1){return s==null||Number.isNaN(s)?"---":Number(s).toFixed(e)}function Tw(s){return s==null||Number.isNaN(s)?0:Math.max(0,Math.min(1,s))}function Id(s,e){return s==null||Number.isNaN(s)||!e?"ok":e.low_bad?e.alarm!==void 0&&s<=e.alarm?"alarm":e.warn!==void 0&&s<=e.warn?"warn":"ok":e.alarm!==void 0&&s>=e.alarm?"alarm":e.warn!==void 0&&s>=e.warn?"warn":"ok"}function Ud(s){return s==="alarm"?"#ff4d4f":s==="warn"?"#f5c542":"#3dd68c"}function Fd(s,e){if(!e||s===null||s===void 0||Number.isNaN(s))return 0;const t=e.min??0,r=e.max??1;return r===t?0:Tw((s-t)/(r-t))}function Aw({attitude:s}){const e=Ke.useRef(null),t=Ke.useRef(null),r=Ke.useRef(null);Ke.useEffect(()=>{const u=t.current,c=e.current,f=new pw({canvas:u,antialias:!0,alpha:!1});f.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),f.setScissorTest(!0),f.setClearColor(658963,1);const h=new f_,m=new vw(10406399,1712684,1.1);h.add(m);const g=new Sw(16777215,.6);g.position.set(2,3,1),h.add(g);const y=new Qr;y.rotation.x=-Math.PI/2,h.add(y);const v=Pw();v.add(new ww(.28)),y.add(v),h.add(new Ew(1.6,8,2766146,1712684));const M=Cw(),T=new Ks(-1.15,1.15,1.15,-1.15,.1,10),S=new $n(42,1,.05,20),x=new Q(.85,.55,.85),_=new Mw().setFromVector3(x),P={theta:_.theta,phi:_.phi,radius:_.radius};Df(S,P);const R=.55,L=new Ks(-R,R,R,-R,.05,20);L.position.set(0,1.4,0),L.up.set(-1,0,0),L.lookAt(0,0,0);const Y=new Ks(-R,R,R,-R,.05,20);Y.position.set(1.4,0,0),Y.up.set(0,1,0),Y.lookAt(0,0,0);const U=new Ks(-R,R,R,-R,.05,20);U.position.set(0,0,1.4),U.up.set(0,1,0),U.lookAt(0,0,0),U.scale.x=-1;const N={on:!1,x:0,y:0};r.current={renderer:f,scene:h,vehicle:v,cameras:{persp:S,top:L,side:U,front:Y},gizmo:M,gizmoCam:T,wrap:c,orbit:P,drag:N};const H=te=>{Nf(te,c)&&(N.on=!0,N.x=te.clientX,N.y=te.clientY,c.setPointerCapture(te.pointerId),c.classList.add("orbiting"))},b=te=>{if(c.style.cursor=Nf(te,c)||N.on?"grab":"default",!N.on)return;const ue=te.clientX-N.x,se=te.clientY-N.y;N.x=te.clientX,N.y=te.clientY,P.theta-=ue*.008,P.phi=Sg(P.phi-se*.008,.08,Math.PI-.08),Df(S,P)},w=te=>{if(N.on){N.on=!1;try{c.releasePointerCapture(te.pointerId)}catch{}c.classList.remove("orbiting")}},I=te=>{Nf(te,c)&&(te.preventDefault(),P.radius=Sg(P.radius*(te.deltaY>0?1.08:.92),.45,4),Df(S,P))};c.addEventListener("pointerdown",H),c.addEventListener("pointermove",b),c.addEventListener("pointerup",w),c.addEventListener("pointercancel",w),c.addEventListener("wheel",I,{passive:!1});let K=0;const V=()=>{K=requestAnimationFrame(V),Lw(r.current)},Z=new ResizeObserver(()=>Mg(r.current));return Z.observe(c),Mg(r.current),V(),()=>{cancelAnimationFrame(K),Z.disconnect(),c.removeEventListener("pointerdown",H),c.removeEventListener("pointermove",b),c.removeEventListener("pointerup",w),c.removeEventListener("pointercancel",w),c.removeEventListener("wheel",I),f.dispose()}},[]),Ke.useEffect(()=>{var m;const u=(m=r.current)==null?void 0:m.vehicle;if(!u||!s)return;const c=If(s.roll_deg),f=If(s.pitch_deg),h=If(s.yaw_deg);u.rotation.order="ZYX",u.rotation.set(c,f,h)},[s]);const o=`R ${Zn(s==null?void 0:s.roll_deg,1)}  P ${Zn(s==null?void 0:s.pitch_deg,1)}  Y ${Zn(s==null?void 0:s.yaw_deg,1)}`;return O.jsxs("div",{className:"card card-fill",children:[O.jsxs("h2",{children:["Attitude · IMU ",(s==null?void 0:s.source)==="snapshot"?"(10 Hz)":(s==null?void 0:s.source)==="monitor"?"(1 Hz)":""]}),O.jsxs("div",{ref:e,className:"attitude-grid attitude-third-angle",children:[O.jsx("canvas",{ref:t,style:{position:"absolute",inset:0,width:"100%",height:"100%"}}),O.jsx(Vl,{title:"Top · plan",rpy:o}),O.jsx(Vl,{title:"3D · drag",rpy:o}),O.jsx(Vl,{title:"Front",rpy:o}),O.jsx(Vl,{title:"Right · side",rpy:o})]})]})}function Vl({title:s,rpy:e}){return O.jsxs("div",{className:"view-box",style:{background:"transparent",pointerEvents:"none"},children:[O.jsx("div",{className:"view-label",children:s}),O.jsx("div",{className:"view-rpy",children:e})]})}function Nf(s,e){const t=e.getBoundingClientRect(),r=s.clientX-t.left,o=s.clientY-t.top;return r>=t.width/2&&o>=0&&o<t.height/2}function Df(s,e){s.position.setFromSphericalCoords(e.radius,e.phi,e.theta),s.lookAt(0,0,0),s.updateProjectionMatrix()}function Sg(s,e,t){return Math.max(e,Math.min(t,s))}function If(s){return s==null||Number.isNaN(s)?0:s*Math.PI/180}function Cw(){const s=new f_,e=new Qr;e.rotation.x=-Math.PI/2,s.add(e);const t=.85,r=[{dir:[1,0,0],color:16731471,label:"X"},{dir:[0,1,0],color:4052620,label:"Y"},{dir:[0,0,1],color:5030911,label:"Z"}];for(const o of r){const u=new Hn().setFromPoints([new Q(0,0,0),new Q(o.dir[0]*t,o.dir[1]*t,o.dir[2]*t)]),c=new p_(u,new mu({color:o.color,depthTest:!1,transparent:!0}));c.renderOrder=10,e.add(c);const f=new zn(new gu(.07,.18,8),new Pd({color:o.color,depthTest:!1}));f.position.set(o.dir[0]*t,o.dir[1]*t,o.dir[2]*t),o.dir[0]?f.rotation.z=-Math.PI/2:o.dir[1]||(f.rotation.x=Math.PI/2),f.renderOrder=11,e.add(f);const h=Rw(o.label,o.color);h.position.set(o.dir[0]*(t+.22),o.dir[1]*(t+.22),o.dir[2]*(t+.22)),h.renderOrder=12,e.add(h)}return s}function Rw(s,e){const t=document.createElement("canvas");t.width=64,t.height=64;const r=t.getContext("2d");r.clearRect(0,0,64,64),r.font="bold 44px IBM Plex Sans, sans-serif",r.textAlign="center",r.textBaseline="middle",r.lineWidth=6,r.strokeStyle="#070b10",r.strokeText(s,32,34),r.fillStyle=`#${e.toString(16).padStart(6,"0")}`,r.fillText(s,32,34);const o=new _w(t);o.colorSpace=ui;const u=new d_({map:o,depthTest:!1,depthWrite:!1,transparent:!0}),c=new gw(u);return c.scale.set(.4,.4,.4),c}function bw(s,e){const t=e.position.clone();t.lengthSq()<1e-8?t.set(0,0,1):t.normalize(),s.position.copy(t.multiplyScalar(2.4)),s.up.copy(e.up),s.lookAt(0,0,0),s.updateProjectionMatrix()}function Pw(){const s=new Qr,e=new zn(new ns(.42,.18,.14),new Hl({color:4034521,metalness:.2,roughness:.5}));s.add(e);const t=new zn(new gu(.07,.14,12),new Hl({color:16106818}));t.rotation.z=-Math.PI/2,t.position.x=.26,s.add(t);const r=new zn(new ns(.08,.04,.02),new Hl({color:16731471}));r.position.set(.05,.11,.04),s.add(r);const o=r.clone();return o.material=new Hl({color:4052620}),o.position.set(.05,-.11,.04),s.add(o),s}function Mg(s){if(!s)return;const{wrap:e,renderer:t,cameras:r}=s,o=Math.max(1,e.clientWidth),u=Math.max(1,e.clientHeight);t.setSize(o,u,!1);const c=o/2/Math.max(1,u/2);r.persp.aspect=c,r.persp.updateProjectionMatrix()}function Lw(s){if(!s)return;const{renderer:e,scene:t,cameras:r,gizmo:o,gizmoCam:u,wrap:c}=s,f=c.clientWidth,h=c.clientHeight;if(f<2||h<2)return;const m=f/2,g=h/2,y=[{cam:r.top,x:0,y:g,ww:m,hh:g},{cam:r.persp,x:m,y:g,ww:m,hh:g},{cam:r.front,x:0,y:0,ww:m,hh:g},{cam:r.side,x:m,y:0,ww:m,hh:g}];for(const v of y){e.setViewport(v.x,v.y,v.ww,v.hh),e.setScissor(v.x,v.y,v.ww,v.hh),e.render(t,v.cam);const M=Math.max(36,Math.min(v.ww,v.hh)*.32),T=Math.max(4,M*.06),S=v.x+T,x=v.y+T;bw(u,v.cam),e.clearDepth(),e.setViewport(S,x,M,M),e.setScissor(S,x,M,M),e.render(o,u)}}const __="kankai.ui.v1";function Od(){try{const s=localStorage.getItem(__);if(!s)return{};const e=JSON.parse(s);return e&&typeof e=="object"?e:{}}catch{return{}}}function eu(s){try{const e={...Od(),...s,updatedAt:Date.now()};return localStorage.setItem(__,JSON.stringify(e)),e}catch{return null}}const Jr=9,Nw=Array.from({length:Jr*Jr},(s,e)=>[e%Jr,Math.floor(e/Jr)]),kd=.1,io=10;function Eg(s){const e=Number(s)||0;return e<=.05?io:Math.min(io,Math.max(kd,1/e))}function Dw(s){const e=Number(s);return!Number.isFinite(e)||e>=io-1e-6?0:1/Math.max(kd,e)}function Iw(s,e){return(Number(e)||0)<=.05||s>=io-1e-6?"∞":Number(s).toFixed(2)}function Uw(s,e){const t=1/Jr,r=1/Jr;return[s*t,e*r,t,r].map(o=>Number(o.toFixed(4)))}function Fw(s){if(!s)return null;const e=String(s).split(",").map(Number);return e.length!==4||e.some(t=>Number.isNaN(t))?null:e}function Ow(s,e){if(!s||!e)return!1;const t=.5/Jr;return s.every((r,o)=>Math.abs(r-e[o])<t)}function kw(s){if(!s)return!0;const[e,t,r,o]=s;return e<=.02&&t<=.02&&e+r>=.98&&t+o>=.98}async function Bw(s,e){const t=await fetch(`/api/cameras/${encodeURIComponent(s)}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(e)}),r=await t.json().catch(()=>({}));if(!t.ok)throw new Error(r.error||`camera ${t.status}`);return r}function wg(s,e){const t=String(e||"").replace(/^\/+|\/+$/g,"");try{const r=new URL(s||"http://127.0.0.1:8889");return r.hostname=window.location.hostname||r.hostname,r.pathname=`/${t}/`,r.search="",r.hash="",r.toString()}catch{return`http://${window.location.hostname||"127.0.0.1"}:8889/${t}/`}}function Tg(s){const e=Number(s);if(!Number.isFinite(e)||e<0)return"—";const t=["B","KB","MB","GB","TB"];let r=e,o=0;for(;r>=1e3&&o<t.length-1;)r/=1e3,o+=1;const u=r>=10||o===0?0:1;return`${r.toFixed(u)}${t[o]}`}function zw(s,e){var t,r;return!!((r=(t=((s==null?void 0:s.state)||[]).find(o=>o.id===e))==null?void 0:t.recording)!=null&&r.active)}function Hw({config:s}){var he,fe,k,ce,ke,J,de,Se,Me,Ie,Ge,Qe,Et,B,wt,gt,ne,ie,De,Ae,Ue,F,A,oe,ve,Ee,ge,Ze,Ne,Be,ct,Te,Oe,_t,rt,Ve,lt,ft,Ct,j,Ce,pe,me,Re,Je,vt;const e=(s==null?void 0:s.cameras)||{},t=Array.isArray(e.items)&&e.items.length?e.items:[{id:"cam0",path:"cam0",nickname:"Ceiling"},{id:"cam1",path:"cam1",nickname:"Canopy"}],r=Array.isArray(e.zoom)&&e.zoom.length?e.zoom:[{id:"binned",label:"1x Wide"},{id:"full",label:"1x Full res"},{id:"crop",label:"1.5x Crop"}],o=e.webrtc||"http://127.0.0.1:8889",[u,c]=Ke.useState(()=>Od().camId||"cam0"),[f,h]=Ke.useState(null),[m,g]=Ke.useState(!1),[y,v]=Ke.useState(""),M=xe=>{c(xe),eu({camId:xe})};Ke.useEffect(()=>{var xe;if(!t.some(at=>at.id===u)){const at=((xe=t[0])==null?void 0:xe.id)||"cam0";c(at)}},[t,u]);const T=async()=>{try{const at=await(await fetch("/api/cameras")).json();h(at),at.error&&v(at.error)}catch(xe){v(String(xe.message||xe))}};Ke.useEffect(()=>{T();const xe=setInterval(T,2500);return()=>clearInterval(xe)},[]);const S=Ke.useMemo(()=>((f==null?void 0:f.state)||[]).find(xe=>xe.id===u)||{},[f,u]),x=new Set(Array.isArray(e.live)?e.live:["exposure","wb"]),_=xe=>!x.has(xe),P=xe=>_(xe)?"restart":"live",R=(f==null?void 0:f.backend)||e.backend||"mediamtx",L=(f==null?void 0:f.webrtc)||o,Y=Fw((he=S.focus)==null?void 0:he.window),U=kw(Y),N=((fe=S.focus)==null?void 0:fe.mode)==="continuous"||((k=S.focus)==null?void 0:k.mode)==="auto",H=((ce=S.exposure)==null?void 0:ce.mode)==="me"?"me":"ae",b=H==="ae",w=H==="me",I=((ke=S.wb)==null?void 0:ke.mode)==="manual",K=!I,[V,Z]=Ke.useState({shutter_us:8e3,gain:2}),[te,ue]=Ke.useState([2,1.5]);Ke.useEffect(()=>{var tt,kt,qt,En;const xe=Number(((tt=S.exposure)==null?void 0:tt.seed_shutter_us)??((kt=S.exposure)==null?void 0:kt.shutter_us)),at=Number(((qt=S.exposure)==null?void 0:qt.seed_gain)??((En=S.exposure)==null?void 0:En.gain));Number.isFinite(xe)&&xe>0&&Number.isFinite(at)&&at>0&&Z({shutter_us:xe,gain:at})},[(J=S.exposure)==null?void 0:J.mode,(de=S.exposure)==null?void 0:de.shutter_us,(Se=S.exposure)==null?void 0:Se.gain,(Me=S.exposure)==null?void 0:Me.seed_shutter_us,(Ie=S.exposure)==null?void 0:Ie.seed_gain]),Ke.useEffect(()=>{var kt,qt,En;const xe=(qt=(kt=S.wb)==null?void 0:kt.seed_gains)!=null&&qt[0]?S.wb.seed_gains:(En=S.wb)==null?void 0:En.gains,at=Number(xe==null?void 0:xe[0]),tt=Number(xe==null?void 0:xe[1]);Number.isFinite(at)&&at>0&&Number.isFinite(tt)&&tt>0&&ue([at,tt])},[(Ge=S.wb)==null?void 0:Ge.mode,(Qe=S.wb)==null?void 0:Qe.gains,(Et=S.wb)==null?void 0:Et.seed_gains]);const se=async(xe,at)=>{g(!0),v(at?"Restarting stream…":"Applying…");try{const tt=await Bw(u,xe);v(tt.restart?"Stream restarted":"Applied live"),await T()}catch(tt){v(String(tt.message||tt))}finally{g(!1)}},G=xe=>{const at=xe.path||xe.id;window.open(wg(L,at),"_blank","noopener,noreferrer")};return O.jsxs("div",{className:"card control-card cam-panel",children:[O.jsx("h2",{children:"Cameras"}),O.jsx("div",{className:"cam-notebook",role:"tablist","aria-label":"Cameras",children:t.map(xe=>{const at=(xe.nickname||xe.id||"").toLowerCase(),tt=at.includes("canopy")?"canopy":at.includes("ceil")?"ceiling":xe.id==="cam1"?"canopy":"ceiling",kt=zw(f,xe.id);return O.jsxs("button",{type:"button",role:"tab","aria-selected":xe.id===u,className:`cam-notebook-tab ${tt}${xe.id===u?" on":""}`,onClick:()=>M(xe.id),children:[xe.nickname||xe.id,kt?O.jsx("span",{className:"cam-rec-dot",title:"Recording","aria-label":"Recording"}):null]},xe.id)})}),O.jsxs("div",{className:"cam-page",role:"tabpanel",children:[O.jsxs("div",{className:"cam-page-bar",children:[O.jsxs("span",{className:`lamp ${S.ready?"live":"stale"}`,children:[O.jsx("i",{})," ",S.ready?"ready":"idle"]}),(B=S.recording)!=null&&B.active?O.jsxs("span",{className:`lamp rec on${f!=null&&f.storage&&!f.storage.ok?" alarm":""}`,title:"Recording to disk",children:[O.jsx("i",{})," REC"]}):null,f!=null&&f.storage?O.jsxs("span",{className:`cam-free${f.storage.ok?"":" low"}`,title:`Keep at least ${Tg(f.storage.min_free_bytes)} free`,children:[Tg(f.storage.free_bytes)," free"]}):null,O.jsx("span",{className:"cam-note",children:R}),O.jsx("a",{className:"btn cam-stream",href:wg(L,S.path||u),target:"_blank",rel:"noopener noreferrer",title:`Open WebRTC stream (${S.path||u})`,onClick:xe=>{xe.preventDefault();const at=t.find(tt=>tt.id===u)||{id:u,path:S.path||u};G(at)},children:"Stream"})]}),O.jsxs("div",{className:"cam-section",children:[O.jsxs("div",{className:"cam-label",children:["Zoom (sensor mode · ",P("zoom"),")"]}),O.jsx("div",{className:"cam-seg",children:r.map(xe=>O.jsx("button",{type:"button",disabled:m,className:S.zoom===xe.id?"btn primary":"btn",title:xe.hint||(xe.width&&xe.height?`Sensor ${xe.sensor_mode||"—"} → stream ${xe.width}x${xe.height} @ ${xe.fps||"—"} fps`:void 0),onClick:()=>se({zoom:xe.id},_("zoom")),children:xe.label},xe.id))})]}),O.jsxs("div",{className:"cam-grid",children:[O.jsxs("div",{className:"cam-col cam-col-focus",children:[O.jsxs("div",{className:"cam-label",children:["Focus (",P("focus"),")"]}),O.jsxs("div",{className:"cam-seg",children:[O.jsx("button",{type:"button",disabled:m,className:((wt=S.focus)==null?void 0:wt.mode)==="continuous"?"btn primary":"btn",onClick:()=>se({focus:{mode:"continuous"}},_("focus")),children:"AF cont."}),O.jsx("button",{type:"button",disabled:m,className:((gt=S.focus)==null?void 0:gt.mode)==="auto"?"btn primary":"btn",onClick:()=>se({focus:{mode:"auto"}},_("focus")),children:"AF once"}),O.jsx("button",{type:"button",disabled:m,className:((ne=S.focus)==null?void 0:ne.mode)==="manual"?"btn primary":"btn",onClick:()=>{var xe;return se({focus:{mode:"manual",lens_position:((xe=S.focus)==null?void 0:xe.lens_position)||1}},_("focus"))},title:"Manual focus — set focus distance",children:"MF"})]}),O.jsxs("div",{className:"cam-label",children:["AF window",N?U?" · full frame":" · region":""]}),O.jsx("div",{className:`af-grid${N?" af-mode":""}${U?" af-full":""}`,children:Nw.map(([xe,at])=>{const tt=Uw(xe,at),kt=!U&&Ow(Y,tt);return O.jsx("button",{type:"button",disabled:m,className:kt?"af-cell on":"af-cell","aria-pressed":kt,title:`AF cell ${xe+1},${at+1}`,onClick:()=>se({focus:{window:tt}},_("focus"))},`${xe}-${at}`)})}),O.jsx("button",{type:"button",className:`btn cam-wide${U?" primary":""}`,disabled:m,"aria-pressed":U,onClick:()=>se({focus:{window:""}},_("focus")),children:"Full frame"}),((ie=S.focus)==null?void 0:ie.mode)==="manual"&&O.jsx(qs,{label:"Focus",unit:"m",display:Iw(Eg((De=S.focus)==null?void 0:De.lens_position),(Ae=S.focus)==null?void 0:Ae.lens_position),digits:2,min:kd,max:io,step:.05,disabled:m,value:Eg((Ue=S.focus)==null?void 0:Ue.lens_position),onCommit:xe=>se({focus:{mode:"manual",lens_position:Dw(xe)}},_("focus"))})]}),O.jsxs("div",{className:"cam-col cam-col-exp",children:[O.jsxs("div",{className:"cam-label",children:["Exposure (",w?"manual":"auto"," · ",P("exposure"),")"]}),O.jsxs("div",{className:"cam-seg",children:[O.jsx("button",{type:"button",disabled:m,className:b?"btn primary":"btn",onClick:()=>se({exposure:{mode:"ae"}},!0),children:"Auto"}),O.jsx("button",{type:"button",disabled:m,className:w?"btn primary":"btn",onClick:()=>se({exposure:{mode:"me"}},_("exposure")),children:"Manual"})]}),O.jsx(qs,{label:"EV",digits:1,min:-4,max:4,step:.5,disabled:m||!b,value:Number(((F=S.exposure)==null?void 0:F.ev)??0),onCommit:xe=>se({exposure:{mode:"ae",ev:xe}},_("exposure"))}),O.jsx(qs,{label:"Shutter",unit:"µs",digits:0,min:100,max:33e3,step:100,disabled:m||!w,value:Number(w?((A=S.exposure)==null?void 0:A.shutter_us)??((oe=S.exposure)==null?void 0:oe.seed_shutter_us)??V.shutter_us:((ve=S.exposure)==null?void 0:ve.seed_shutter_us)??((Ee=S.exposure)==null?void 0:Ee.shutter_us)??V.shutter_us),onCommit:xe=>se({exposure:{mode:"me",shutter_us:xe,gain:V.gain}},_("exposure"))}),O.jsx(qs,{label:"Gain",digits:1,min:1,max:12,step:.1,disabled:m||!w,value:Number(w?((ge=S.exposure)==null?void 0:ge.gain)??((Ze=S.exposure)==null?void 0:Ze.seed_gain)??V.gain:((Ne=S.exposure)==null?void 0:Ne.seed_gain)??((Be=S.exposure)==null?void 0:Be.gain)??V.gain),onCommit:xe=>se({exposure:{mode:"me",shutter_us:V.shutter_us,gain:xe}},_("exposure"))}),O.jsxs("div",{className:"cam-label",children:["White balance (",I?"manual":"auto"," · ",P("wb"),")"]}),O.jsxs("div",{className:"cam-seg",children:[O.jsx("button",{type:"button",disabled:m,className:K?"btn primary":"btn",onClick:()=>se({wb:{mode:"auto"}},_("wb")),children:"Auto"}),O.jsx("button",{type:"button",disabled:m,className:I?"btn primary":"btn",onClick:()=>se({wb:{mode:"manual"}},_("wb")),children:"Manual"})]}),O.jsx(qs,{label:"R",digits:2,min:.5,max:8,step:.05,disabled:m||!I,value:Number(I?((Te=(ct=S.wb)==null?void 0:ct.gains)==null?void 0:Te[0])??((_t=(Oe=S.wb)==null?void 0:Oe.seed_gains)==null?void 0:_t[0])??te[0]:((Ve=(rt=S.wb)==null?void 0:rt.seed_gains)==null?void 0:Ve[0])??((ft=(lt=S.wb)==null?void 0:lt.gains)==null?void 0:ft[0])??te[0]),onCommit:xe=>se({wb:{mode:"manual",gains:[xe,te[1]]}},_("wb"))}),O.jsx(qs,{label:"B",digits:2,min:.5,max:8,step:.05,disabled:m||!I,value:Number(I?((j=(Ct=S.wb)==null?void 0:Ct.gains)==null?void 0:j[1])??((pe=(Ce=S.wb)==null?void 0:Ce.seed_gains)==null?void 0:pe[1])??te[1]:((Re=(me=S.wb)==null?void 0:me.seed_gains)==null?void 0:Re[1])??((vt=(Je=S.wb)==null?void 0:Je.gains)==null?void 0:vt[1])??te[1]),onCommit:xe=>se({wb:{mode:"manual",gains:[te[0],xe]}},_("wb"))})]})]}),O.jsx("div",{className:"cam-note",children:y||`${S.width||"—"}x${S.height||"—"} @ ${S.fps||"—"} fps`})]})]})}function qs({label:s,value:e,min:t,max:r,step:o,digits:u,unit:c="",display:f,disabled:h,onCommit:m}){const[g,y]=Ke.useState(e);Ke.useEffect(()=>{y(e)},[e]);const v=f!=null?g===e?f:Number(g)>=r-1e-6?"∞":Number(g).toFixed(u):Number(g).toFixed(u);return O.jsxs("label",{className:`cam-slider${h?" disabled":""}`,children:[s," ",v,c&&v!=="∞"?` ${c}`:"",O.jsx("input",{type:"range",min:t,max:r,step:o,disabled:h,value:g,onChange:M=>{h||y(Number(M.target.value))},onPointerUp:M=>{h||m(Number(M.currentTarget.value))},onKeyUp:M=>{h||(M.key==="Enter"||M.key==="ArrowLeft"||M.key==="ArrowRight")&&m(Number(M.currentTarget.value))}})]})}const ia=[{key:"x",label:"X"},{key:"y",label:"Y"},{key:"z",label:"Z"},{key:"yaw",label:"Yaw"},{key:"pitch",label:"Pitch"},{key:"roll",label:"Roll"}],ra=[{key:"grab",label:"Grip"},{key:"roll",label:"Roll"}],$s={x:0,y:0,z:0,yaw:0,pitch:0,roll:0},Yr={grab:0,roll:0},Gl=Object.fromEntries(ia.map(s=>[s.key,!0])),Wl=Object.fromEntries(ia.map(s=>[s.key,!1])),jl=Object.fromEntries(ra.map(s=>[s.key,!0])),Xl=Object.fromEntries(ra.map(s=>[s.key,!1])),Vw=40;function yi(s,e,t){return Math.max(e,Math.min(t,s))}function Uf(s){return yi(s,-100,100)/100}function Ff(s,e=-100,t=100){return yi((Number(s)||0)*100,e,t)}function Of(s){return Math.round(s)}function Gw(s){return ia.some(e=>Math.abs(Number(s==null?void 0:s[e.key])||0)>1e-4)}function Ag(s){return ra.some(e=>Math.abs(Number(s==null?void 0:s[e.key])||0)>1e-4)}function Cg(){const e=Od().hand;return!e||typeof e!="object"?Yr:{grab:yi(Number(e.grab)||0,-1,1),roll:yi(Number(e.roll)||0,-1,1)}}async function Yl(s){const e=await fetch("/api/command",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(s)});if(!e.ok)throw new Error(`command ${e.status}`);return e.json()}function Ww({command:s,lights:e,stepPercent:t=5,children:r}){const o=Array.isArray(e)&&e.length?e:[],[u,c]=Ke.useState($s),[f,h]=Ke.useState(Gl),[m,g]=Ke.useState(Cg),[y,v]=Ke.useState(jl),[M,T]=Ke.useState(()=>Object.fromEntries(o.map(ne=>[ne,0]))),[S,x]=Ke.useState(()=>Object.fromEntries(o.map(ne=>[ne,!1]))),_=Ke.useRef({twist:!1,hand:!1,lights:!1}),P=Ke.useRef({twist:!1,hand:!1,lights:!1}),R=Ke.useRef({twist:!1,hand:!1,lights:!1}),L=Ke.useRef({twist:null,hand:null,lights:null}),Y=Ke.useRef(S);Y.current=S;const U=Ke.useRef(f);U.current=f;const N=Ke.useRef(y);N.current=y;const H=Ke.useRef({twist:u,hand:m,lights:M});H.current={twist:u,hand:m,lights:M};const b=ne=>{!P.current[ne]&&!R.current[ne]&&L.current[ne]==null&&(_.current[ne]=!1)},w=(ne,ie)=>{const De={};for(const Ae of o)De[Ae]=ie[Ae]?ne[Ae]??0:0;return De},I=(ne,ie)=>{const De={};for(const Ae of ia)De[Ae.key]=ie[Ae.key]?ne[Ae.key]??0:0;return De},K=(ne,ie)=>{const De={};for(const Ae of ra)De[Ae.key]=ie[Ae.key]?ne[Ae.key]??0:0;return De};Ke.useEffect(()=>{T(ne=>{const ie={};for(const De of o)ie[De]=ne[De]??0;return ie}),x(ne=>{const ie={};for(const De of o)ie[De]=!!ne[De];return ie})},[e]),Ke.useEffect(()=>{s&&(!_.current.twist&&s.twist&&c(ne=>{const ie={...ne};let De=!1;for(const Ae of ia){if(s.twist[Ae.key]==null)continue;const Ue=yi(Number(s.twist[Ae.key])||0,-1,1);Math.abs(Ue)>1e-4&&ie[Ae.key]!==Ue&&(ie[Ae.key]=Ue,De=!0)}return De?ie:ne}),!_.current.hand&&s.hand&&g(ne=>{const ie={...ne};let De=!1;for(const Ae of ra){if(s.hand[Ae.key]==null)continue;const Ue=yi(Number(s.hand[Ae.key])||0,-1,1);Math.abs(Ue)>1e-4&&ie[Ae.key]!==Ue&&(ie[Ae.key]=Ue,De=!0)}return De?ie:ne}),!_.current.lights&&s.lights&&(T(ne=>{const ie={...ne};let De=!1;for(const Ae of o){if(s.lights[Ae]==null)continue;const Ue=yi(Number(s.lights[Ae])||0,0,1);Ue>0&&ie[Ae]!==Ue&&(ie[Ae]=Ue,De=!0)}return De?ie:ne}),x(ne=>{const ie={...ne};let De=!1;for(const Ae of o){if(s.lights[Ae]==null)continue;const Ue=(Number(s.lights[Ae])||0)>.001;!!ie[Ae]!==Ue&&(ie[Ae]=Ue,De=!0)}return De?ie:ne})))},[s,e,o]);const V=Ke.useCallback((ne,ie,De={})=>{if(De.keepalive||(_.current[ne]=!0),L.current[ne]=ie,R.current[ne])return;R.current[ne]=!0,(async()=>{for(;L.current[ne]!=null;){const Ue=L.current[ne];L.current[ne]=null;try{ne==="twist"?await Yl({twist:Ue}):ne==="hand"?await Yl({hand:Ue}):await Yl({lights:Ue})}catch{}}R.current[ne]=!1,b(ne)})()},[]);Ke.useEffect(()=>{const ne=setInterval(()=>{if(typeof document<"u"&&document.hidden)return;const Ae=I(H.current.twist,U.current);Gw(Ae)&&V("twist",Ae,{keepalive:!0});const Ue=K(H.current.hand,N.current);Ag(Ue)&&V("hand",Ue,{keepalive:!0})},Vw),ie=()=>{var Ue;const Ae=JSON.stringify({twist:$s,hand:Yr});try{(Ue=navigator.sendBeacon)==null||Ue.call(navigator,"/api/command",new Blob([Ae],{type:"application/json"}))}catch{}},De=()=>{typeof document<"u"&&document.hidden&&ie()};return window.addEventListener("pagehide",ie),document.addEventListener("visibilitychange",De),()=>{clearInterval(ne),window.removeEventListener("pagehide",ie),document.removeEventListener("visibilitychange",De)}},[V]);const Z=(ne,ie)=>{const De={...H.current.twist,[ne]:Uf(ie)};H.current.twist=De,c(De),V("twist",I(De,U.current))},te=ne=>{const ie={...U.current,[ne]:!1};U.current=ie,h(ie),V("twist",I(H.current.twist,ie))},ue=ne=>{const ie={...U.current,[ne]:!0};U.current=ie,h(ie),V("twist",I(H.current.twist,ie))},se=()=>{U.current=Wl,h(Wl),P.current.twist=!1,L.current.twist=null,V("twist",{...$s})},G=()=>{U.current=Gl,h(Gl),P.current.twist=!1,L.current.twist=null,V("twist",I(H.current.twist,Gl))},he=()=>{const ne={...$s};H.current.twist=ne,c(ne),V("twist",I(ne,U.current))},fe=(ne,ie)=>{const De={...H.current.hand,[ne]:Uf(ie)};H.current.hand=De,g(De),eu({hand:De}),V("hand",K(De,N.current))},k=ne=>{const ie={...N.current,[ne]:!1};N.current=ie,v(ie),V("hand",K(H.current.hand,ie))},ce=ne=>{const ie={...N.current,[ne]:!0};N.current=ie,v(ie),V("hand",K(H.current.hand,ie))},ke=()=>{N.current=Xl,v(Xl),P.current.hand=!1,L.current.hand=null,V("hand",{...Yr})},J=()=>{N.current=jl,v(jl),P.current.hand=!1,L.current.hand=null,V("hand",K(H.current.hand,jl))},de=()=>{const ne={...Yr};H.current.hand=ne,g(ne),eu({hand:ne}),V("hand",K(ne,N.current))},Se=(ne,ie)=>{const De=Uf(ie),Ae={...H.current.lights,[ne]:De};T(Ae),H.current.lights=Ae;const Ue={...Y.current};V("lights",w(Ae,Ue))},Me=ne=>{const ie={...Y.current,[ne]:!1};Y.current=ie,x(ie),V("lights",w(H.current.lights,ie))},Ie=ne=>{const ie={...H.current.lights};ie[ne]>0||(ie[ne]=1);const De={...Y.current,[ne]:!0};Y.current=De,T(ie),H.current.lights=ie,x(De),V("lights",w(ie,De))},Ge=()=>{_.current.twist=!0,_.current.hand=!0,P.current.twist=!1,P.current.hand=!1,L.current.twist=null,L.current.hand=null,c($s),H.current.twist={...$s},U.current=Wl,h(Wl),g(Yr),H.current.hand={...Yr},N.current=Xl,v(Xl),eu({hand:Yr}),R.current.twist=!0,R.current.hand=!0,Yl({estop:!0}).finally(()=>{R.current.twist=!1,R.current.hand=!1,b("twist"),b("hand")})},Qe=()=>{const ne=Object.fromEntries(o.map(ie=>[ie,!1]));Y.current=ne,x(ne),P.current.lights=!1,L.current.lights=null,V("lights",w(H.current.lights,ne))},Et=()=>{const ne=Object.fromEntries(o.map(De=>[De,1])),ie=Object.fromEntries(o.map(De=>[De,!0]));Y.current=ie,T(ne),H.current.lights=ne,x(ie),P.current.lights=!1,L.current.lights=null,V("lights",w(ne,ie))},B=()=>{const ne=Object.fromEntries(o.map(ie=>[ie,0]));T(ne),H.current.lights=ne,V("lights",w(ne,Y.current))},wt=(ne,ie)=>{P.current[ne]=ie,ie?_.current[ne]=!0:b(ne)},gt=Ke.useRef(!1);return Ke.useEffect(()=>{if(gt.current)return;gt.current=!0;const ne=Cg();H.current.hand=ne,Ag(ne)&&V("hand",K(ne,N.current))},[V]),O.jsxs("div",{className:"control-dock",children:[O.jsxs("div",{className:"control-top",children:[O.jsxs("div",{className:"control-lights-wrap",children:[O.jsxs("div",{className:"card control-card",children:[O.jsx("h2",{children:"Thrusters"}),O.jsx("div",{className:"fader-row",children:ia.map(ne=>O.jsx(kf,{label:ne.label,value:Ff(u[ne.key]),min:-100,max:100,step:t,bipolar:!0,outputSwitch:!0,lit:!!f[ne.key],onChange:ie=>Z(ne.key,ie),onOff:()=>te(ne.key),onOn:()=>ue(ne.key),onActive:ie=>wt("twist",ie)},ne.key))})]}),O.jsxs("div",{className:"lights-bulk",children:[O.jsxs("button",{className:"lights-on",type:"button",onClick:G,children:[O.jsx("span",{children:"All on"}),O.jsx("b",{children:"I"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:se,children:[O.jsx("span",{children:"All off"}),O.jsx("b",{children:"O"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:he,children:[O.jsx("span",{children:"All reset"}),O.jsx("b",{children:"0"})]})]})]}),O.jsxs("div",{className:"control-lights-wrap",children:[O.jsxs("div",{className:"card control-card",children:[O.jsx("h2",{children:"Manipulators"}),O.jsx("div",{className:"fader-row",children:ra.map(ne=>O.jsx(kf,{label:ne.label,value:Ff(m[ne.key]),min:-100,max:100,step:t,bipolar:!0,outputSwitch:!0,lit:!!y[ne.key],onChange:ie=>fe(ne.key,ie),onOff:()=>k(ne.key),onOn:()=>ce(ne.key),onActive:ie=>wt("hand",ie)},ne.key))})]}),O.jsxs("div",{className:"lights-bulk",children:[O.jsxs("button",{className:"lights-on",type:"button",onClick:J,children:[O.jsx("span",{children:"All on"}),O.jsx("b",{children:"I"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:ke,children:[O.jsx("span",{children:"All off"}),O.jsx("b",{children:"O"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:de,children:[O.jsx("span",{children:"All reset"}),O.jsx("b",{children:"0"})]})]})]}),O.jsxs("button",{className:"estop",type:"button",onClick:Ge,children:[O.jsx("span",{children:"E-STOP"}),O.jsx("b",{children:"N"})]})]}),O.jsxs("div",{className:"control-lights-wrap",children:[O.jsxs("div",{className:"card control-card control-lights",children:[O.jsx("h2",{children:"Lights"}),O.jsx("div",{className:"fader-row",children:o.map(ne=>O.jsx(kf,{label:ne,value:Ff(M[ne]??0,0,100),min:0,max:100,step:t,outputSwitch:!0,lit:!!S[ne],onChange:ie=>Se(ne,ie),onOff:()=>Me(ne),onOn:()=>Ie(ne),onActive:ie=>wt("lights",ie)},ne))})]}),O.jsxs("div",{className:"lights-bulk",children:[O.jsxs("button",{className:"lights-on",type:"button",onClick:Et,children:[O.jsx("span",{children:"All on"}),O.jsx("b",{children:"100"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:Qe,children:[O.jsx("span",{children:"All off"}),O.jsx("b",{children:"O"})]}),O.jsxs("button",{className:"lights-off",type:"button",onClick:B,children:[O.jsx("span",{children:"All reset"}),O.jsx("b",{children:"0"})]})]})]}),r]})}function kf({label:s,value:e,min:t,max:r,step:o,bipolar:u=!1,outputSwitch:c=!1,lit:f=!1,onChange:h,onOff:m,onOn:g,onActive:y}){const v=Ke.useRef(null),M=Ke.useRef(null),T=Ke.useRef(null),S=Of(yi(e,t,r)),x=Ke.useRef(S);x.current=S;const _=r-t||1,P=(S-t)/_,R=u?(0-t)/_:0,L=(1-Math.max(P,R))*100,Y=Math.min(P,R)*100,U=(1-P)*100,N=Z=>{const te=v.current;if(!te)return;const ue=te.getBoundingClientRect(),se=yi((Z-ue.top)/ue.height,0,1);h(Of(r-se*_))},H=Z=>{const te=Of(yi(x.current+Z*o,t,r));x.current=te,h(te)},b=Z=>{y==null||y(!0),w(),H(Z),M.current=setTimeout(()=>{M.current=null,T.current=setInterval(()=>H(Z),70)},280)},w=()=>{M.current&&(clearTimeout(M.current),M.current=null),T.current&&(clearInterval(T.current),T.current=null)},I=Z=>{Z.preventDefault(),y==null||y(!0),Z.currentTarget.setPointerCapture(Z.pointerId),N(Z.clientY)},K=Z=>{Z.currentTarget.hasPointerCapture(Z.pointerId)&&N(Z.clientY)},V=()=>{w(),y==null||y(!1)};return Ke.useEffect(()=>()=>{M.current&&clearTimeout(M.current),T.current&&clearInterval(T.current)},[]),O.jsxs("div",{className:`fader${c&&f?" lit":""}`,children:[O.jsxs("div",{className:"fader-val",children:[S,"%"]}),O.jsx("button",{type:"button",className:"fader-btn","aria-label":`${s} plus`,onPointerDown:Z=>{Z.preventDefault(),Z.currentTarget.setPointerCapture(Z.pointerId),b(1)},onPointerUp:V,onPointerCancel:V,onLostPointerCapture:V,children:"+"}),O.jsxs("div",{className:"fader-track",ref:v,role:"slider","aria-label":s,"aria-valuemin":t,"aria-valuemax":r,"aria-valuenow":S,onPointerDown:I,onPointerMove:K,onPointerUp:V,onPointerCancel:V,children:[u&&O.jsx("i",{className:"fader-zero"}),O.jsx("div",{className:"fader-fill",style:{top:`${L}%`,bottom:`${Y}%`}}),O.jsx("div",{className:"fader-knob",style:{top:`${U}%`}})]}),O.jsx("button",{type:"button",className:"fader-btn","aria-label":`${s} minus`,onPointerDown:Z=>{Z.preventDefault(),Z.currentTarget.setPointerCapture(Z.pointerId),b(-1)},onPointerUp:V,onPointerCancel:V,onLostPointerCapture:V,children:"−"}),c?O.jsxs("div",{className:"fader-switch",children:[O.jsx("button",{type:"button",className:`fader-on-btn${f?" active":""}`,"aria-label":`${s} on`,onClick:()=>{y==null||y(!0),g==null||g(),y==null||y(!1)},children:"On"}),O.jsx("button",{type:"button",className:`fader-off-btn${f?"":" active"}`,"aria-label":`${s} off`,onClick:()=>{y==null||y(!0),m==null||m(),y==null||y(!1)},children:"Off"})]}):null,O.jsx("button",{type:"button",className:"fader-zero-btn","aria-label":`${s} reset 0`,onClick:()=>{y==null||y(!0),h(0),y==null||y(!1)},children:"0"}),O.jsx("div",{className:"fader-label",children:s})]})}const v_=[0,.25,.5,.75,1],Rg=160,bg=148;function Bd(s,e){const t=(s==null?void 0:s.min)??0,r=(s==null?void 0:s.max)??1;return t+e*(r-t)}function jw(s,e){return 0}function li({label:s,unit:e,value:t,spec:r,digits:o=1}){const u=Id(t,r),c=t==null?"#8b9bb0":Ud(u),f=Fd(t,r),h=Math.PI*.75,m=Math.PI*2.25,g=h+f*(m-h),y=80,v=68,M=48,T=Qs(y,v,M-8,g),S=Yw(y,v,M,h,m),x=jw(),_=v_.map(P=>{const R=h+P*(m-h),L=Qs(y,v,M+2,R),Y=Qs(y,v,M-9,R),U=Qs(y,v,M+18,R);return{outer:L,inner:Y,labelPos:U,major:P===0||P===.5||P===1,text:Zn(Bd(r,P),x)}});return O.jsxs("div",{className:"meter",children:[O.jsxs("div",{className:"meter-dial",children:[O.jsxs("svg",{viewBox:`0 0 ${Rg} ${bg}`,"aria-label":s,children:[O.jsx("path",{d:S,fill:"none",stroke:"#0a0e13",strokeWidth:"12",strokeLinecap:"round"}),O.jsx("path",{d:S,fill:"none",stroke:c,strokeWidth:"12",strokeLinecap:"round",strokeDasharray:`${f*Lg(M,h,m)} ${Lg(M,h,m)}`}),_.map((P,R)=>O.jsx("line",{x1:P.inner.x,y1:P.inner.y,x2:P.outer.x,y2:P.outer.y,stroke:"#5a6a7a",strokeWidth:P.major?1.6:1},R)),O.jsx("line",{x1:y,y1:v,x2:T.x,y2:T.y,stroke:"#f4f7fb",strokeWidth:"2.5"}),O.jsx("circle",{cx:y,cy:v,r:"4",fill:"#f4f7fb"})]}),O.jsx("div",{className:"dial-tick-labels","aria-hidden":"true",children:_.filter(P=>P.major).map((P,R)=>O.jsx("span",{className:"scale-tick",style:{left:`${P.labelPos.x/Rg*100}%`,top:`${P.labelPos.y/bg*100}%`},children:P.text},R))})]}),O.jsxs("div",{className:"val",style:{color:c},children:[Zn(t,o),O.jsx("span",{className:"unit",children:e})]}),O.jsx("div",{className:"lbl",children:s})]})}function Pg({label:s,unit:e,value:t,spec:r,digits:o=1,tickDigits:u=0}){const c=Id(t,r),f=t==null?"#8b9bb0":Ud(c),h=`${Fd(t,r)*100}%`,m=v_.map(g=>({p:g,text:Zn(Bd(r,g),u),major:g===0||g===.5||g===1}));return O.jsxs("div",{className:"bar",children:[O.jsx("div",{children:s}),O.jsxs("div",{className:"bar-scale",children:[O.jsxs("div",{className:"track","aria-label":s,children:[O.jsxs("div",{className:"bar-ticks","aria-hidden":"true",children:[O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{})]}),O.jsx("div",{className:"fill",style:{width:h,background:f}})]}),O.jsx("div",{className:"bar-tick-labels","aria-hidden":"true",children:m.map((g,y)=>O.jsx("span",{className:`scale-tick${g.major?" major":""}`,style:{left:`${g.p*100}%`},children:g.major?g.text:""},y))})]}),O.jsxs("div",{style:{color:f,textAlign:"right"},children:[Zn(t,o)," ",e]})]})}function Xw({nickname:s,value:e,spec:t}){const r=Id(e,t),o=e==null?"#8b9bb0":Ud(r),u=`${Fd(e,t)*100}%`,c=0,f=[0,.5,1].map(h=>({p:h,text:Zn(Bd(t,h),c)}));return O.jsxs("div",{className:"thermo",children:[O.jsxs("div",{className:"thermo-body",children:[O.jsx("div",{className:"thermo-tick-labels","aria-hidden":"true",children:f.map(h=>O.jsx("span",{className:"scale-tick",style:{bottom:`${h.p*100}%`},children:h.text},h.p))}),O.jsxs("div",{className:"well","aria-label":s,children:[O.jsxs("div",{className:"thermo-ticks","aria-hidden":"true",children:[O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{}),O.jsx("i",{})]}),O.jsx("div",{className:"fill",style:{height:u,background:o}})]})]}),O.jsxs("div",{className:"thermo-caption",children:[O.jsx("div",{className:"name",children:s}),O.jsxs("div",{className:"val",style:{color:o},children:[Zn(e,1)," °C"]})]})]})}function Qs(s,e,t,r){return{x:s+t*Math.cos(r),y:e+t*Math.sin(r)}}function Yw(s,e,t,r,o){const u=Qs(s,e,t,r),c=Qs(s,e,t,o),f=o-r>Math.PI?1:0;return`M ${u.x} ${u.y} A ${t} ${t} 0 ${f} 1 ${c.x} ${c.y}`}function Lg(s,e,t){return s*(t-e)}const qw={monitor:null,attitude:null,health:{publisher:"never",imu:"never"}};function $w(){var b;const[s,e]=Ke.useState(null),[t,r]=Ke.useState(qw),[o,u]=Ke.useState("connecting"),[c,f]=Ke.useState(!0),[h,m]=Ke.useState(!1),[g,y]=Ke.useState(!1),v=Ke.useRef(null),M=Ke.useRef({leak:!1,publisher:"never",stream:"connecting"}),T=Ke.useRef(null);Ke.useEffect(()=>{const w=_m.loadPrefs();m(w.quietAdvisories);const I=new _m;I.armed=!0,I.quietAdvisories=w.quietAdvisories,v.current=I,I.arm().then(()=>f(!0)).catch(()=>{});const K=()=>{I.arm().then(()=>{f(!0),I.update(M.current)}).catch(()=>{})};return window.addEventListener("pointerdown",K,{once:!0}),window.addEventListener("keydown",K,{once:!0}),fetch("/api/config").then(V=>V.json()).then(e).catch(()=>e({temperatures:[],gauges:{}})),()=>{window.removeEventListener("pointerdown",K),window.removeEventListener("keydown",K)}},[]),Ke.useEffect(()=>{const w=new EventSource("/api/stream");let I=null;const K=()=>{I&&(clearTimeout(I),I=null),u("connected")};return w.addEventListener("state",V=>{K();try{const Z=JSON.parse(V.data);r(Z),Z.config&&e(Z.config)}catch{}}),w.onerror=()=>{I||(I=setTimeout(()=>{I=null,u("disconnected")},3500))},()=>{I&&clearTimeout(I),w.close()}},[]);const S=t.monitor,x=t.health||{},_=!!(S!=null&&S.water_ch0_detected||S!=null&&S.water_ch1_detected||x.leak),P=(s==null?void 0:s.gauges)||{},R=(s==null?void 0:s.temperatures)||[],L=(s==null?void 0:s.temperature_gauge)||{min:0,max:80},Y=(s==null?void 0:s.leaks)||[];M.current={leak:_,publisher:x.publisher,stream:o};const U=_?{cls:"alarm",text:"Siren · water leak"}:h?{cls:"quiet",text:"Advisories off (siren on)"}:o==="disconnected"?{cls:"stale",text:"Pulse · stream lost (~5s)"}:x.publisher==="stale"||x.publisher==="never"?{cls:"stale",text:"Beep · publisher stale (~20s)"}:{cls:"quiet",text:"Alerts quiet"};Ke.useEffect(()=>{var w;(w=v.current)==null||w.update({leak:_,publisher:x.publisher,stream:o})},[_,x.publisher,o,c,h]);const N=()=>{var I;const w=!h;m(w),(I=v.current)==null||I.setQuietAdvisories(w)};Ke.useEffect(()=>{const w=()=>y(!!document.fullscreenElement);return document.addEventListener("fullscreenchange",w),()=>document.removeEventListener("fullscreenchange",w)},[]);const H=async()=>{var w;try{if(document.fullscreenElement)await document.exitFullscreen();else{const I=T.current||document.documentElement;await((w=I.requestFullscreen)==null?void 0:w.call(I))}}catch{}};return O.jsxs("div",{className:"app",ref:T,children:[O.jsxs("header",{className:"header",children:[O.jsx("div",{className:"brand",children:"Kankai Control Panel"}),O.jsx(Bf,{name:"Link",status:o==="connected"?"live":o}),O.jsx(Bf,{name:"Publisher",status:x.publisher||"never"}),O.jsx(Bf,{name:"IMU",status:x.imu||"never"}),O.jsxs("div",{className:"meta",children:[O.jsx("span",{children:(S==null?void 0:S.stamp_jst)||"---"}),O.jsxs("span",{children:["seq ",(S==null?void 0:S.seq)??"---"]}),O.jsxs("span",{children:["elapsed ",(S==null?void 0:S.elapsed_hms)||"---"]})]}),O.jsxs("div",{className:"header-actions",children:[O.jsxs("span",{className:`lamp ${U.cls}`,title:"Siren = water leak (never muted). Soft pulse ~5s = data stream lost. Soft beep ~20s = publisher stale.",children:[O.jsx("i",{})," ",U.text]}),O.jsx("button",{className:h?"btn":"btn armed",onClick:N,title:"Turn off advisory pulse/beep only. Leak siren always stays on.",children:h?"Advisories off":"Advisories on"}),O.jsx("button",{className:"btn",type:"button",onClick:H,children:g?"Exit full":"Fullscreen"})]})]}),o==="disconnected"&&O.jsx("div",{className:"banner",children:"Data stream disconnected — check publisher / node"}),o==="connected"&&(x.publisher==="stale"||x.publisher==="never")&&O.jsx("div",{className:"banner",children:"rov/monitor_value stalled — check monitor_value_pub"}),O.jsxs("div",{className:"layout",children:[O.jsxs("div",{className:"col",children:[O.jsx(Zw,{monitor:S,leak:_,leaks:Y}),O.jsxs("div",{className:"card",children:[O.jsx("h2",{children:"Depth"}),O.jsxs("div",{className:"gauge-row",children:[O.jsx(li,{label:"Depth (seawater)",unit:"m",value:S==null?void 0:S.depth_m,spec:P.depth_m,digits:2}),O.jsx(li,{label:"Raw",unit:"atm",value:S==null?void 0:S.depth_pressure_atm,spec:P.depth_pressure_atm,digits:3}),O.jsx(li,{label:"Temp",unit:"°C",value:S==null?void 0:S.depth_temp_c,spec:L,digits:1})]})]}),O.jsxs("div",{className:"card card-fill",children:[O.jsx("h2",{children:"Temperature"}),O.jsx("div",{className:"thermo-row",children:R.filter(w=>w.key!=="depth_temp_c").map(w=>O.jsx(Xw,{nickname:w.nickname||w.key,value:S?S[w.key]:null,spec:L},w.key))})]}),O.jsxs("div",{className:"card",children:[O.jsx("h2",{children:"Hull interior"}),O.jsxs("div",{className:"bars",children:[O.jsx(Pg,{label:"Humidity",unit:"%",value:S==null?void 0:S.bme_humidity_percent,spec:P.bme_humidity_percent}),O.jsx(Pg,{label:"Pressure",unit:"atm",value:S==null?void 0:S.bme_pressure_atm,spec:P.bme_pressure_atm,digits:3,tickDigits:1})]})]})]}),O.jsxs("div",{className:"col col-right",children:[O.jsx(Aw,{attitude:t.attitude}),O.jsxs("div",{className:"card card-compact",children:[O.jsx("h2",{children:"Power"}),O.jsxs("div",{className:"gauge-row gauge-row-4",children:[O.jsx(li,{label:"Voltage",unit:"V",value:S==null?void 0:S.voltage_v,spec:P.voltage_v,digits:1}),O.jsx(li,{label:"Current",unit:"A",value:S==null?void 0:S.current_a,spec:P.current_a,digits:2}),O.jsx(li,{label:"Power",unit:"W",value:S==null?void 0:S.power_w,spec:P.power_w,digits:1}),O.jsx(li,{label:"Remaining",unit:"%",value:S==null?void 0:S.remaining_percent,spec:P.remaining_percent,digits:0})]}),O.jsxs("div",{className:"kv",style:{marginTop:8},children:[O.jsx("span",{children:"Energy"}),O.jsxs("b",{children:[Zn(S==null?void 0:S.accumulated_energy_wh,2)," Wh"]}),O.jsx("span",{children:"Peak"}),O.jsxs("b",{children:[Zn(S==null?void 0:S.peak_power_w,1)," W"]})]})]}),O.jsxs("div",{className:"card card-compact",children:[O.jsx("h2",{children:"RPi"}),O.jsxs("div",{className:"gauge-row gauge-row-4",children:[O.jsx(li,{label:"CPU",unit:"%",value:S==null?void 0:S.rpi_cpu_util_percent,spec:P.rpi_cpu_util_percent,digits:1}),O.jsx(li,{label:"GPU",unit:"%",value:S==null?void 0:S.rpi_gpu_util_percent,spec:P.rpi_gpu_util_percent,digits:1}),O.jsx(li,{label:"Fan(PWM)",unit:"%",value:S==null?void 0:S.rpi_fan_pwm_percent,spec:P.rpi_fan_pwm_percent,digits:0}),O.jsx(li,{label:"Fan",unit:"rpm",value:S==null?void 0:S.rpi_fan_rpm,spec:P.rpi_fan_rpm,digits:0})]})]})]}),O.jsx(Ww,{command:t.command,lights:s==null?void 0:s.lights,stepPercent:((b=s==null?void 0:s.command)==null?void 0:b.step_percent)??5,children:O.jsx(Hw,{config:s})})]})]})}function Bf({name:s,status:e}){const t=e==="live"?"live":e==="stale"||e==="reconnecting"||e==="connecting"?"stale":"disconnected";return O.jsxs("span",{className:`lamp ${t}`,children:[O.jsx("i",{})," ",s," ",Kw(e)]})}function Kw(s){return s==="live"?"live":s==="stale"?"stale":s==="never"?"none":s==="connecting"?"connecting":s==="disconnected"?"disconnected":s}function Zw({monitor:s,leak:e,leaks:t}){const r=Object.fromEntries((t||[]).map(o=>[Number(o.channel),o.nickname]));return O.jsxs("div",{className:e?"card alarm":"card",children:[O.jsx("h2",{children:"Water Leakage"}),O.jsxs("div",{className:"leak-grid",children:[O.jsx(Ng,{n:r[0]||"ch0",v:s==null?void 0:s.water_ch0_probe_v,hot:s==null?void 0:s.water_ch0_detected}),O.jsx(Ng,{n:r[1]||"ch1",v:s==null?void 0:s.water_ch1_probe_v,hot:s==null?void 0:s.water_ch1_detected})]})]})}function Ng({n:s,v:e,hot:t}){return O.jsxs("div",{className:t?"ch hot blink":"ch",children:[O.jsx("div",{className:"name",children:s}),O.jsxs("div",{className:"big",children:[Zn(e,2)," V"]}),O.jsx("div",{className:"status",children:t?"DETECTED":"ok"})]})}l0.createRoot(document.getElementById("root")).render(O.jsx(n0.StrictMode,{children:O.jsx($w,{})}));
