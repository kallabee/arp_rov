(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const o of document.querySelectorAll('link[rel="modulepreload"]'))r(o);new MutationObserver(o=>{for(const u of o)if(u.type==="childList")for(const c of u.addedNodes)c.tagName==="LINK"&&c.rel==="modulepreload"&&r(c)}).observe(document,{childList:!0,subtree:!0});function t(o){const u={};return o.integrity&&(u.integrity=o.integrity),o.referrerPolicy&&(u.referrerPolicy=o.referrerPolicy),o.crossOrigin==="use-credentials"?u.credentials="include":o.crossOrigin==="anonymous"?u.credentials="omit":u.credentials="same-origin",u}function r(o){if(o.ep)return;o.ep=!0;const u=t(o);fetch(o.href,u)}})();function Wv(s){return s&&s.__esModule&&Object.prototype.hasOwnProperty.call(s,"default")?s.default:s}var zc={exports:{}},Ha={},Hc={exports:{}},gt={};/**
 * @license React
 * react.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var tm;function Xv(){if(tm)return gt;tm=1;var s=Symbol.for("react.element"),e=Symbol.for("react.portal"),t=Symbol.for("react.fragment"),r=Symbol.for("react.strict_mode"),o=Symbol.for("react.profiler"),u=Symbol.for("react.provider"),c=Symbol.for("react.context"),d=Symbol.for("react.forward_ref"),h=Symbol.for("react.suspense"),m=Symbol.for("react.memo"),g=Symbol.for("react.lazy"),y=Symbol.iterator;function v(F){return F===null||typeof F!="object"?null:(F=y&&F[y]||F["@@iterator"],typeof F=="function"?F:null)}var M={isMounted:function(){return!1},enqueueForceUpdate:function(){},enqueueReplaceState:function(){},enqueueSetState:function(){}},T=Object.assign,S={};function x(F,ce,Ie){this.props=F,this.context=ce,this.refs=S,this.updater=Ie||M}x.prototype.isReactComponent={},x.prototype.setState=function(F,ce){if(typeof F!="object"&&typeof F!="function"&&F!=null)throw Error("setState(...): takes an object of state variables to update or a function which returns an object of state variables.");this.updater.enqueueSetState(this,F,ce,"setState")},x.prototype.forceUpdate=function(F){this.updater.enqueueForceUpdate(this,F,"forceUpdate")};function _(){}_.prototype=x.prototype;function P(F,ce,Ie){this.props=F,this.context=ce,this.refs=S,this.updater=Ie||M}var R=P.prototype=new _;R.constructor=P,T(R,x.prototype),R.isPureReactComponent=!0;var L=Array.isArray,$=Object.prototype.hasOwnProperty,O={current:null},D={key:!0,ref:!0,__self:!0,__source:!0};function j(F,ce,Ie){var te,fe={},xe=null,Me=null;if(ce!=null)for(te in ce.ref!==void 0&&(Me=ce.ref),ce.key!==void 0&&(xe=""+ce.key),ce)$.call(ce,te)&&!D.hasOwnProperty(te)&&(fe[te]=ce[te]);var Le=arguments.length-2;if(Le===1)fe.children=Ie;else if(1<Le){for(var ke=Array(Le),Ye=0;Ye<Le;Ye++)ke[Ye]=arguments[Ye+2];fe.children=ke}if(F&&F.defaultProps)for(te in Le=F.defaultProps,Le)fe[te]===void 0&&(fe[te]=Le[te]);return{$$typeof:s,type:F,key:xe,ref:Me,props:fe,_owner:O.current}}function b(F,ce){return{$$typeof:s,type:F.type,key:ce,ref:F.ref,props:F.props,_owner:F._owner}}function w(F){return typeof F=="object"&&F!==null&&F.$$typeof===s}function I(F){var ce={"=":"=0",":":"=2"};return"$"+F.replace(/[=:]/g,function(Ie){return ce[Ie]})}var Y=/\/+/g;function K(F,ce){return typeof F=="object"&&F!==null&&F.key!=null?I(""+F.key):ce.toString(36)}function oe(F,ce,Ie,te,fe){var xe=typeof F;(xe==="undefined"||xe==="boolean")&&(F=null);var Me=!1;if(F===null)Me=!0;else switch(xe){case"string":case"number":Me=!0;break;case"object":switch(F.$$typeof){case s:case e:Me=!0}}if(Me)return Me=F,fe=fe(Me),F=te===""?"."+K(Me,0):te,L(fe)?(Ie="",F!=null&&(Ie=F.replace(Y,"$&/")+"/"),oe(fe,ce,Ie,"",function(Ye){return Ye})):fe!=null&&(w(fe)&&(fe=b(fe,Ie+(!fe.key||Me&&Me.key===fe.key?"":(""+fe.key).replace(Y,"$&/")+"/")+F)),ce.push(fe)),1;if(Me=0,te=te===""?".":te+":",L(F))for(var Le=0;Le<F.length;Le++){xe=F[Le];var ke=te+K(xe,Le);Me+=oe(xe,ce,Ie,ke,fe)}else if(ke=v(F),typeof ke=="function")for(F=ke.call(F),Le=0;!(xe=F.next()).done;)xe=xe.value,ke=te+K(xe,Le++),Me+=oe(xe,ce,Ie,ke,fe);else if(xe==="object")throw ce=String(F),Error("Objects are not valid as a React child (found: "+(ce==="[object Object]"?"object with keys {"+Object.keys(F).join(", ")+"}":ce)+"). If you meant to render a collection of children, use an array instead.");return Me}function ne(F,ce,Ie){if(F==null)return F;var te=[],fe=0;return oe(F,te,"","",function(xe){return ce.call(Ie,xe,fe++)}),te}function B(F){if(F._status===-1){var ce=F._result;ce=ce(),ce.then(function(Ie){(F._status===0||F._status===-1)&&(F._status=1,F._result=Ie)},function(Ie){(F._status===0||F._status===-1)&&(F._status=2,F._result=Ie)}),F._status===-1&&(F._status=0,F._result=ce)}if(F._status===1)return F._result.default;throw F._result}var G={current:null},k={transition:null},ue={ReactCurrentDispatcher:G,ReactCurrentBatchConfig:k,ReactCurrentOwner:O};function le(){throw Error("act(...) is not supported in production builds of React.")}return gt.Children={map:ne,forEach:function(F,ce,Ie){ne(F,function(){ce.apply(this,arguments)},Ie)},count:function(F){var ce=0;return ne(F,function(){ce++}),ce},toArray:function(F){return ne(F,function(ce){return ce})||[]},only:function(F){if(!w(F))throw Error("React.Children.only expected to receive a single React element child.");return F}},gt.Component=x,gt.Fragment=t,gt.Profiler=o,gt.PureComponent=P,gt.StrictMode=r,gt.Suspense=h,gt.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=ue,gt.act=le,gt.cloneElement=function(F,ce,Ie){if(F==null)throw Error("React.cloneElement(...): The argument must be a React element, but you passed "+F+".");var te=T({},F.props),fe=F.key,xe=F.ref,Me=F._owner;if(ce!=null){if(ce.ref!==void 0&&(xe=ce.ref,Me=O.current),ce.key!==void 0&&(fe=""+ce.key),F.type&&F.type.defaultProps)var Le=F.type.defaultProps;for(ke in ce)$.call(ce,ke)&&!D.hasOwnProperty(ke)&&(te[ke]=ce[ke]===void 0&&Le!==void 0?Le[ke]:ce[ke])}var ke=arguments.length-2;if(ke===1)te.children=Ie;else if(1<ke){Le=Array(ke);for(var Ye=0;Ye<ke;Ye++)Le[Ye]=arguments[Ye+2];te.children=Le}return{$$typeof:s,type:F.type,key:fe,ref:xe,props:te,_owner:Me}},gt.createContext=function(F){return F={$$typeof:c,_currentValue:F,_currentValue2:F,_threadCount:0,Provider:null,Consumer:null,_defaultValue:null,_globalName:null},F.Provider={$$typeof:u,_context:F},F.Consumer=F},gt.createElement=j,gt.createFactory=function(F){var ce=j.bind(null,F);return ce.type=F,ce},gt.createRef=function(){return{current:null}},gt.forwardRef=function(F){return{$$typeof:d,render:F}},gt.isValidElement=w,gt.lazy=function(F){return{$$typeof:g,_payload:{_status:-1,_result:F},_init:B}},gt.memo=function(F,ce){return{$$typeof:m,type:F,compare:ce===void 0?null:ce}},gt.startTransition=function(F){var ce=k.transition;k.transition={};try{F()}finally{k.transition=ce}},gt.unstable_act=le,gt.useCallback=function(F,ce){return G.current.useCallback(F,ce)},gt.useContext=function(F){return G.current.useContext(F)},gt.useDebugValue=function(){},gt.useDeferredValue=function(F){return G.current.useDeferredValue(F)},gt.useEffect=function(F,ce){return G.current.useEffect(F,ce)},gt.useId=function(){return G.current.useId()},gt.useImperativeHandle=function(F,ce,Ie){return G.current.useImperativeHandle(F,ce,Ie)},gt.useInsertionEffect=function(F,ce){return G.current.useInsertionEffect(F,ce)},gt.useLayoutEffect=function(F,ce){return G.current.useLayoutEffect(F,ce)},gt.useMemo=function(F,ce){return G.current.useMemo(F,ce)},gt.useReducer=function(F,ce,Ie){return G.current.useReducer(F,ce,Ie)},gt.useRef=function(F){return G.current.useRef(F)},gt.useState=function(F){return G.current.useState(F)},gt.useSyncExternalStore=function(F,ce,Ie){return G.current.useSyncExternalStore(F,ce,Ie)},gt.useTransition=function(){return G.current.useTransition()},gt.version="18.3.1",gt}var nm;function pd(){return nm||(nm=1,Hc.exports=Xv()),Hc.exports}/**
 * @license React
 * react-jsx-runtime.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var im;function jv(){if(im)return Ha;im=1;var s=pd(),e=Symbol.for("react.element"),t=Symbol.for("react.fragment"),r=Object.prototype.hasOwnProperty,o=s.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.ReactCurrentOwner,u={key:!0,ref:!0,__self:!0,__source:!0};function c(d,h,m){var g,y={},v=null,M=null;m!==void 0&&(v=""+m),h.key!==void 0&&(v=""+h.key),h.ref!==void 0&&(M=h.ref);for(g in h)r.call(h,g)&&!u.hasOwnProperty(g)&&(y[g]=h[g]);if(d&&d.defaultProps)for(g in h=d.defaultProps,h)y[g]===void 0&&(y[g]=h[g]);return{$$typeof:e,type:d,key:v,ref:M,props:y,_owner:o.current}}return Ha.Fragment=t,Ha.jsx=c,Ha.jsxs=c,Ha}var rm;function Yv(){return rm||(rm=1,zc.exports=jv()),zc.exports}var V=Yv(),it=pd();const qv=Wv(it);var ul={},Vc={exports:{}},Pn={},Gc={exports:{}},Wc={};/**
 * @license React
 * scheduler.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var sm;function $v(){return sm||(sm=1,(function(s){function e(k,ue){var le=k.length;k.push(ue);e:for(;0<le;){var F=le-1>>>1,ce=k[F];if(0<o(ce,ue))k[F]=ue,k[le]=ce,le=F;else break e}}function t(k){return k.length===0?null:k[0]}function r(k){if(k.length===0)return null;var ue=k[0],le=k.pop();if(le!==ue){k[0]=le;e:for(var F=0,ce=k.length,Ie=ce>>>1;F<Ie;){var te=2*(F+1)-1,fe=k[te],xe=te+1,Me=k[xe];if(0>o(fe,le))xe<ce&&0>o(Me,fe)?(k[F]=Me,k[xe]=le,F=xe):(k[F]=fe,k[te]=le,F=te);else if(xe<ce&&0>o(Me,le))k[F]=Me,k[xe]=le,F=xe;else break e}}return ue}function o(k,ue){var le=k.sortIndex-ue.sortIndex;return le!==0?le:k.id-ue.id}if(typeof performance=="object"&&typeof performance.now=="function"){var u=performance;s.unstable_now=function(){return u.now()}}else{var c=Date,d=c.now();s.unstable_now=function(){return c.now()-d}}var h=[],m=[],g=1,y=null,v=3,M=!1,T=!1,S=!1,x=typeof setTimeout=="function"?setTimeout:null,_=typeof clearTimeout=="function"?clearTimeout:null,P=typeof setImmediate<"u"?setImmediate:null;typeof navigator<"u"&&navigator.scheduling!==void 0&&navigator.scheduling.isInputPending!==void 0&&navigator.scheduling.isInputPending.bind(navigator.scheduling);function R(k){for(var ue=t(m);ue!==null;){if(ue.callback===null)r(m);else if(ue.startTime<=k)r(m),ue.sortIndex=ue.expirationTime,e(h,ue);else break;ue=t(m)}}function L(k){if(S=!1,R(k),!T)if(t(h)!==null)T=!0,B($);else{var ue=t(m);ue!==null&&G(L,ue.startTime-k)}}function $(k,ue){T=!1,S&&(S=!1,_(j),j=-1),M=!0;var le=v;try{for(R(ue),y=t(h);y!==null&&(!(y.expirationTime>ue)||k&&!I());){var F=y.callback;if(typeof F=="function"){y.callback=null,v=y.priorityLevel;var ce=F(y.expirationTime<=ue);ue=s.unstable_now(),typeof ce=="function"?y.callback=ce:y===t(h)&&r(h),R(ue)}else r(h);y=t(h)}if(y!==null)var Ie=!0;else{var te=t(m);te!==null&&G(L,te.startTime-ue),Ie=!1}return Ie}finally{y=null,v=le,M=!1}}var O=!1,D=null,j=-1,b=5,w=-1;function I(){return!(s.unstable_now()-w<b)}function Y(){if(D!==null){var k=s.unstable_now();w=k;var ue=!0;try{ue=D(!0,k)}finally{ue?K():(O=!1,D=null)}}else O=!1}var K;if(typeof P=="function")K=function(){P(Y)};else if(typeof MessageChannel<"u"){var oe=new MessageChannel,ne=oe.port2;oe.port1.onmessage=Y,K=function(){ne.postMessage(null)}}else K=function(){x(Y,0)};function B(k){D=k,O||(O=!0,K())}function G(k,ue){j=x(function(){k(s.unstable_now())},ue)}s.unstable_IdlePriority=5,s.unstable_ImmediatePriority=1,s.unstable_LowPriority=4,s.unstable_NormalPriority=3,s.unstable_Profiling=null,s.unstable_UserBlockingPriority=2,s.unstable_cancelCallback=function(k){k.callback=null},s.unstable_continueExecution=function(){T||M||(T=!0,B($))},s.unstable_forceFrameRate=function(k){0>k||125<k?console.error("forceFrameRate takes a positive int between 0 and 125, forcing frame rates higher than 125 fps is not supported"):b=0<k?Math.floor(1e3/k):5},s.unstable_getCurrentPriorityLevel=function(){return v},s.unstable_getFirstCallbackNode=function(){return t(h)},s.unstable_next=function(k){switch(v){case 1:case 2:case 3:var ue=3;break;default:ue=v}var le=v;v=ue;try{return k()}finally{v=le}},s.unstable_pauseExecution=function(){},s.unstable_requestPaint=function(){},s.unstable_runWithPriority=function(k,ue){switch(k){case 1:case 2:case 3:case 4:case 5:break;default:k=3}var le=v;v=k;try{return ue()}finally{v=le}},s.unstable_scheduleCallback=function(k,ue,le){var F=s.unstable_now();switch(typeof le=="object"&&le!==null?(le=le.delay,le=typeof le=="number"&&0<le?F+le:F):le=F,k){case 1:var ce=-1;break;case 2:ce=250;break;case 5:ce=1073741823;break;case 4:ce=1e4;break;default:ce=5e3}return ce=le+ce,k={id:g++,callback:ue,priorityLevel:k,startTime:le,expirationTime:ce,sortIndex:-1},le>F?(k.sortIndex=le,e(m,k),t(h)===null&&k===t(m)&&(S?(_(j),j=-1):S=!0,G(L,le-F))):(k.sortIndex=ce,e(h,k),T||M||(T=!0,B($))),k},s.unstable_shouldYield=I,s.unstable_wrapCallback=function(k){var ue=v;return function(){var le=v;v=ue;try{return k.apply(this,arguments)}finally{v=le}}}})(Wc)),Wc}var am;function Kv(){return am||(am=1,Gc.exports=$v()),Gc.exports}/**
 * @license React
 * react-dom.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */var om;function Zv(){if(om)return Pn;om=1;var s=pd(),e=Kv();function t(n){for(var i="https://reactjs.org/docs/error-decoder.html?invariant="+n,a=1;a<arguments.length;a++)i+="&args[]="+encodeURIComponent(arguments[a]);return"Minified React error #"+n+"; visit "+i+" for the full message or use the non-minified dev environment for full errors and additional helpful warnings."}var r=new Set,o={};function u(n,i){c(n,i),c(n+"Capture",i)}function c(n,i){for(o[n]=i,n=0;n<i.length;n++)r.add(i[n])}var d=!(typeof window>"u"||typeof window.document>"u"||typeof window.document.createElement>"u"),h=Object.prototype.hasOwnProperty,m=/^[:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD][:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\-.0-9\u00B7\u0300-\u036F\u203F-\u2040]*$/,g={},y={};function v(n){return h.call(y,n)?!0:h.call(g,n)?!1:m.test(n)?y[n]=!0:(g[n]=!0,!1)}function M(n,i,a,l){if(a!==null&&a.type===0)return!1;switch(typeof i){case"function":case"symbol":return!0;case"boolean":return l?!1:a!==null?!a.acceptsBooleans:(n=n.toLowerCase().slice(0,5),n!=="data-"&&n!=="aria-");default:return!1}}function T(n,i,a,l){if(i===null||typeof i>"u"||M(n,i,a,l))return!0;if(l)return!1;if(a!==null)switch(a.type){case 3:return!i;case 4:return i===!1;case 5:return isNaN(i);case 6:return isNaN(i)||1>i}return!1}function S(n,i,a,l,f,p,E){this.acceptsBooleans=i===2||i===3||i===4,this.attributeName=l,this.attributeNamespace=f,this.mustUseProperty=a,this.propertyName=n,this.type=i,this.sanitizeURL=p,this.removeEmptyString=E}var x={};"children dangerouslySetInnerHTML defaultValue defaultChecked innerHTML suppressContentEditableWarning suppressHydrationWarning style".split(" ").forEach(function(n){x[n]=new S(n,0,!1,n,null,!1,!1)}),[["acceptCharset","accept-charset"],["className","class"],["htmlFor","for"],["httpEquiv","http-equiv"]].forEach(function(n){var i=n[0];x[i]=new S(i,1,!1,n[1],null,!1,!1)}),["contentEditable","draggable","spellCheck","value"].forEach(function(n){x[n]=new S(n,2,!1,n.toLowerCase(),null,!1,!1)}),["autoReverse","externalResourcesRequired","focusable","preserveAlpha"].forEach(function(n){x[n]=new S(n,2,!1,n,null,!1,!1)}),"allowFullScreen async autoFocus autoPlay controls default defer disabled disablePictureInPicture disableRemotePlayback formNoValidate hidden loop noModule noValidate open playsInline readOnly required reversed scoped seamless itemScope".split(" ").forEach(function(n){x[n]=new S(n,3,!1,n.toLowerCase(),null,!1,!1)}),["checked","multiple","muted","selected"].forEach(function(n){x[n]=new S(n,3,!0,n,null,!1,!1)}),["capture","download"].forEach(function(n){x[n]=new S(n,4,!1,n,null,!1,!1)}),["cols","rows","size","span"].forEach(function(n){x[n]=new S(n,6,!1,n,null,!1,!1)}),["rowSpan","start"].forEach(function(n){x[n]=new S(n,5,!1,n.toLowerCase(),null,!1,!1)});var _=/[\-:]([a-z])/g;function P(n){return n[1].toUpperCase()}"accent-height alignment-baseline arabic-form baseline-shift cap-height clip-path clip-rule color-interpolation color-interpolation-filters color-profile color-rendering dominant-baseline enable-background fill-opacity fill-rule flood-color flood-opacity font-family font-size font-size-adjust font-stretch font-style font-variant font-weight glyph-name glyph-orientation-horizontal glyph-orientation-vertical horiz-adv-x horiz-origin-x image-rendering letter-spacing lighting-color marker-end marker-mid marker-start overline-position overline-thickness paint-order panose-1 pointer-events rendering-intent shape-rendering stop-color stop-opacity strikethrough-position strikethrough-thickness stroke-dasharray stroke-dashoffset stroke-linecap stroke-linejoin stroke-miterlimit stroke-opacity stroke-width text-anchor text-decoration text-rendering underline-position underline-thickness unicode-bidi unicode-range units-per-em v-alphabetic v-hanging v-ideographic v-mathematical vector-effect vert-adv-y vert-origin-x vert-origin-y word-spacing writing-mode xmlns:xlink x-height".split(" ").forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,null,!1,!1)}),"xlink:actuate xlink:arcrole xlink:role xlink:show xlink:title xlink:type".split(" ").forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,"http://www.w3.org/1999/xlink",!1,!1)}),["xml:base","xml:lang","xml:space"].forEach(function(n){var i=n.replace(_,P);x[i]=new S(i,1,!1,n,"http://www.w3.org/XML/1998/namespace",!1,!1)}),["tabIndex","crossOrigin"].forEach(function(n){x[n]=new S(n,1,!1,n.toLowerCase(),null,!1,!1)}),x.xlinkHref=new S("xlinkHref",1,!1,"xlink:href","http://www.w3.org/1999/xlink",!0,!1),["src","href","action","formAction"].forEach(function(n){x[n]=new S(n,1,!1,n.toLowerCase(),null,!0,!0)});function R(n,i,a,l){var f=x.hasOwnProperty(i)?x[i]:null;(f!==null?f.type!==0:l||!(2<i.length)||i[0]!=="o"&&i[0]!=="O"||i[1]!=="n"&&i[1]!=="N")&&(T(i,a,f,l)&&(a=null),l||f===null?v(i)&&(a===null?n.removeAttribute(i):n.setAttribute(i,""+a)):f.mustUseProperty?n[f.propertyName]=a===null?f.type===3?!1:"":a:(i=f.attributeName,l=f.attributeNamespace,a===null?n.removeAttribute(i):(f=f.type,a=f===3||f===4&&a===!0?"":""+a,l?n.setAttributeNS(l,i,a):n.setAttribute(i,a))))}var L=s.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED,$=Symbol.for("react.element"),O=Symbol.for("react.portal"),D=Symbol.for("react.fragment"),j=Symbol.for("react.strict_mode"),b=Symbol.for("react.profiler"),w=Symbol.for("react.provider"),I=Symbol.for("react.context"),Y=Symbol.for("react.forward_ref"),K=Symbol.for("react.suspense"),oe=Symbol.for("react.suspense_list"),ne=Symbol.for("react.memo"),B=Symbol.for("react.lazy"),G=Symbol.for("react.offscreen"),k=Symbol.iterator;function ue(n){return n===null||typeof n!="object"?null:(n=k&&n[k]||n["@@iterator"],typeof n=="function"?n:null)}var le=Object.assign,F;function ce(n){if(F===void 0)try{throw Error()}catch(a){var i=a.stack.trim().match(/\n( *(at )?)/);F=i&&i[1]||""}return`
`+F+n}var Ie=!1;function te(n,i){if(!n||Ie)return"";Ie=!0;var a=Error.prepareStackTrace;Error.prepareStackTrace=void 0;try{if(i)if(i=function(){throw Error()},Object.defineProperty(i.prototype,"props",{set:function(){throw Error()}}),typeof Reflect=="object"&&Reflect.construct){try{Reflect.construct(i,[])}catch(re){var l=re}Reflect.construct(n,[],i)}else{try{i.call()}catch(re){l=re}n.call(i.prototype)}else{try{throw Error()}catch(re){l=re}n()}}catch(re){if(re&&l&&typeof re.stack=="string"){for(var f=re.stack.split(`
`),p=l.stack.split(`
`),E=f.length-1,N=p.length-1;1<=E&&0<=N&&f[E]!==p[N];)N--;for(;1<=E&&0<=N;E--,N--)if(f[E]!==p[N]){if(E!==1||N!==1)do if(E--,N--,0>N||f[E]!==p[N]){var H=`
`+f[E].replace(" at new "," at ");return n.displayName&&H.includes("<anonymous>")&&(H=H.replace("<anonymous>",n.displayName)),H}while(1<=E&&0<=N);break}}}finally{Ie=!1,Error.prepareStackTrace=a}return(n=n?n.displayName||n.name:"")?ce(n):""}function fe(n){switch(n.tag){case 5:return ce(n.type);case 16:return ce("Lazy");case 13:return ce("Suspense");case 19:return ce("SuspenseList");case 0:case 2:case 15:return n=te(n.type,!1),n;case 11:return n=te(n.type.render,!1),n;case 1:return n=te(n.type,!0),n;default:return""}}function xe(n){if(n==null)return null;if(typeof n=="function")return n.displayName||n.name||null;if(typeof n=="string")return n;switch(n){case D:return"Fragment";case O:return"Portal";case b:return"Profiler";case j:return"StrictMode";case K:return"Suspense";case oe:return"SuspenseList"}if(typeof n=="object")switch(n.$$typeof){case I:return(n.displayName||"Context")+".Consumer";case w:return(n._context.displayName||"Context")+".Provider";case Y:var i=n.render;return n=n.displayName,n||(n=i.displayName||i.name||"",n=n!==""?"ForwardRef("+n+")":"ForwardRef"),n;case ne:return i=n.displayName||null,i!==null?i:xe(n.type)||"Memo";case B:i=n._payload,n=n._init;try{return xe(n(i))}catch{}}return null}function Me(n){var i=n.type;switch(n.tag){case 24:return"Cache";case 9:return(i.displayName||"Context")+".Consumer";case 10:return(i._context.displayName||"Context")+".Provider";case 18:return"DehydratedFragment";case 11:return n=i.render,n=n.displayName||n.name||"",i.displayName||(n!==""?"ForwardRef("+n+")":"ForwardRef");case 7:return"Fragment";case 5:return i;case 4:return"Portal";case 3:return"Root";case 6:return"Text";case 16:return xe(i);case 8:return i===j?"StrictMode":"Mode";case 22:return"Offscreen";case 12:return"Profiler";case 21:return"Scope";case 13:return"Suspense";case 19:return"SuspenseList";case 25:return"TracingMarker";case 1:case 0:case 17:case 2:case 14:case 15:if(typeof i=="function")return i.displayName||i.name||null;if(typeof i=="string")return i}return null}function Le(n){switch(typeof n){case"boolean":case"number":case"string":case"undefined":return n;case"object":return n;default:return""}}function ke(n){var i=n.type;return(n=n.nodeName)&&n.toLowerCase()==="input"&&(i==="checkbox"||i==="radio")}function Ye(n){var i=ke(n)?"checked":"value",a=Object.getOwnPropertyDescriptor(n.constructor.prototype,i),l=""+n[i];if(!n.hasOwnProperty(i)&&typeof a<"u"&&typeof a.get=="function"&&typeof a.set=="function"){var f=a.get,p=a.set;return Object.defineProperty(n,i,{configurable:!0,get:function(){return f.call(this)},set:function(E){l=""+E,p.call(this,E)}}),Object.defineProperty(n,i,{enumerable:a.enumerable}),{getValue:function(){return l},setValue:function(E){l=""+E},stopTracking:function(){n._valueTracker=null,delete n[i]}}}}function wt(n){n._valueTracker||(n._valueTracker=Ye(n))}function z(n){if(!n)return!1;var i=n._valueTracker;if(!i)return!0;var a=i.getValue(),l="";return n&&(l=ke(n)?n.checked?"true":"false":n.value),n=l,n!==a?(i.setValue(n),!0):!1}function bt(n){if(n=n||(typeof document<"u"?document:void 0),typeof n>"u")return null;try{return n.activeElement||n.body}catch{return n.body}}function vt(n,i){var a=i.checked;return le({},i,{defaultChecked:void 0,defaultValue:void 0,value:void 0,checked:a??n._wrapperState.initialChecked})}function yt(n,i){var a=i.defaultValue==null?"":i.defaultValue,l=i.checked!=null?i.checked:i.defaultChecked;a=Le(i.value!=null?i.value:a),n._wrapperState={initialChecked:l,initialValue:a,controlled:i.type==="checkbox"||i.type==="radio"?i.checked!=null:i.value!=null}}function We(n,i){i=i.checked,i!=null&&R(n,"checked",i,!1)}function Lt(n,i){We(n,i);var a=Le(i.value),l=i.type;if(a!=null)l==="number"?(a===0&&n.value===""||n.value!=a)&&(n.value=""+a):n.value!==""+a&&(n.value=""+a);else if(l==="submit"||l==="reset"){n.removeAttribute("value");return}i.hasOwnProperty("value")?rt(n,i.type,a):i.hasOwnProperty("defaultValue")&&rt(n,i.type,Le(i.defaultValue)),i.checked==null&&i.defaultChecked!=null&&(n.defaultChecked=!!i.defaultChecked)}function tt(n,i,a){if(i.hasOwnProperty("value")||i.hasOwnProperty("defaultValue")){var l=i.type;if(!(l!=="submit"&&l!=="reset"||i.value!==void 0&&i.value!==null))return;i=""+n._wrapperState.initialValue,a||i===n.value||(n.value=i),n.defaultValue=i}a=n.name,a!==""&&(n.name=""),n.defaultChecked=!!n._wrapperState.initialChecked,a!==""&&(n.name=a)}function rt(n,i,a){(i!=="number"||bt(n.ownerDocument)!==n)&&(a==null?n.defaultValue=""+n._wrapperState.initialValue:n.defaultValue!==""+a&&(n.defaultValue=""+a))}var U=Array.isArray;function A(n,i,a,l){if(n=n.options,i){i={};for(var f=0;f<a.length;f++)i["$"+a[f]]=!0;for(a=0;a<n.length;a++)f=i.hasOwnProperty("$"+n[a].value),n[a].selected!==f&&(n[a].selected=f),f&&l&&(n[a].defaultSelected=!0)}else{for(a=""+Le(a),i=null,f=0;f<n.length;f++){if(n[f].value===a){n[f].selected=!0,l&&(n[f].defaultSelected=!0);return}i!==null||n[f].disabled||(i=n[f])}i!==null&&(i.selected=!0)}}function se(n,i){if(i.dangerouslySetInnerHTML!=null)throw Error(t(91));return le({},i,{value:void 0,defaultValue:void 0,children:""+n._wrapperState.initialValue})}function _e(n,i){var a=i.value;if(a==null){if(a=i.children,i=i.defaultValue,a!=null){if(i!=null)throw Error(t(92));if(U(a)){if(1<a.length)throw Error(t(93));a=a[0]}i=a}i==null&&(i=""),a=i}n._wrapperState={initialValue:Le(a)}}function ye(n,i){var a=Le(i.value),l=Le(i.defaultValue);a!=null&&(a=""+a,a!==n.value&&(n.value=a),i.defaultValue==null&&n.defaultValue!==a&&(n.defaultValue=a)),l!=null&&(n.defaultValue=""+l)}function me(n){var i=n.textContent;i===n._wrapperState.initialValue&&i!==""&&i!==null&&(n.value=i)}function je(n){switch(n){case"svg":return"http://www.w3.org/2000/svg";case"math":return"http://www.w3.org/1998/Math/MathML";default:return"http://www.w3.org/1999/xhtml"}}function be(n,i){return n==null||n==="http://www.w3.org/1999/xhtml"?je(i):n==="http://www.w3.org/2000/svg"&&i==="foreignObject"?"http://www.w3.org/1999/xhtml":n}var De,ot=(function(n){return typeof MSApp<"u"&&MSApp.execUnsafeLocalFunction?function(i,a,l,f){MSApp.execUnsafeLocalFunction(function(){return n(i,a,l,f)})}:n})(function(n,i){if(n.namespaceURI!=="http://www.w3.org/2000/svg"||"innerHTML"in n)n.innerHTML=i;else{for(De=De||document.createElement("div"),De.innerHTML="<svg>"+i.valueOf().toString()+"</svg>",i=De.firstChild;n.firstChild;)n.removeChild(n.firstChild);for(;i.firstChild;)n.appendChild(i.firstChild)}});function Ee(n,i){if(i){var a=n.firstChild;if(a&&a===n.lastChild&&a.nodeType===3){a.nodeValue=i;return}}n.textContent=i}var Ne={animationIterationCount:!0,aspectRatio:!0,borderImageOutset:!0,borderImageSlice:!0,borderImageWidth:!0,boxFlex:!0,boxFlexGroup:!0,boxOrdinalGroup:!0,columnCount:!0,columns:!0,flex:!0,flexGrow:!0,flexPositive:!0,flexShrink:!0,flexNegative:!0,flexOrder:!0,gridArea:!0,gridRow:!0,gridRowEnd:!0,gridRowSpan:!0,gridRowStart:!0,gridColumn:!0,gridColumnEnd:!0,gridColumnSpan:!0,gridColumnStart:!0,fontWeight:!0,lineClamp:!0,lineHeight:!0,opacity:!0,order:!0,orphans:!0,tabSize:!0,widows:!0,zIndex:!0,zoom:!0,fillOpacity:!0,floodOpacity:!0,stopOpacity:!0,strokeDasharray:!0,strokeDashoffset:!0,strokeMiterlimit:!0,strokeOpacity:!0,strokeWidth:!0},pt=["Webkit","ms","Moz","O"];Object.keys(Ne).forEach(function(n){pt.forEach(function(i){i=i+n.charAt(0).toUpperCase()+n.substring(1),Ne[i]=Ne[n]})});function Je(n,i,a){return i==null||typeof i=="boolean"||i===""?"":a||typeof i!="number"||i===0||Ne.hasOwnProperty(n)&&Ne[n]?(""+i).trim():i+"px"}function Oe(n,i){n=n.style;for(var a in i)if(i.hasOwnProperty(a)){var l=a.indexOf("--")===0,f=Je(a,i[a],l);a==="float"&&(a="cssFloat"),l?n.setProperty(a,f):n[a]=f}}var st=le({menuitem:!0},{area:!0,base:!0,br:!0,col:!0,embed:!0,hr:!0,img:!0,input:!0,keygen:!0,link:!0,meta:!0,param:!0,source:!0,track:!0,wbr:!0});function lt(n,i){if(i){if(st[n]&&(i.children!=null||i.dangerouslySetInnerHTML!=null))throw Error(t(137,n));if(i.dangerouslySetInnerHTML!=null){if(i.children!=null)throw Error(t(60));if(typeof i.dangerouslySetInnerHTML!="object"||!("__html"in i.dangerouslySetInnerHTML))throw Error(t(61))}if(i.style!=null&&typeof i.style!="object")throw Error(t(62))}}function Tt(n,i){if(n.indexOf("-")===-1)return typeof i.is=="string";switch(n){case"annotation-xml":case"color-profile":case"font-face":case"font-face-src":case"font-face-uri":case"font-face-format":case"font-face-name":case"missing-glyph":return!1;default:return!0}}var X=null;function we(n){return n=n.target||n.srcElement||window,n.correspondingUseElement&&(n=n.correspondingUseElement),n.nodeType===3?n.parentNode:n}var de=null,he=null,Te=null;function qe(n){if(n=Aa(n)){if(typeof de!="function")throw Error(t(280));var i=n.stateNode;i&&(i=wo(i),de(n.stateNode,n.type,i))}}function pe(n){he?Te?Te.push(n):Te=[n]:he=n}function Ke(){if(he){var n=he,i=Te;if(Te=he=null,qe(n),i)for(n=0;n<i.length;n++)qe(i[n])}}function mt(n,i){return n(i)}function ut(){}var Yt=!1;function qt(n,i,a){if(Yt)return n(i,a);Yt=!0;try{return mt(n,i,a)}finally{Yt=!1,(he!==null||Te!==null)&&(ut(),Ke())}}function Vi(n,i){var a=n.stateNode;if(a===null)return null;var l=wo(a);if(l===null)return null;a=l[i];e:switch(i){case"onClick":case"onClickCapture":case"onDoubleClick":case"onDoubleClickCapture":case"onMouseDown":case"onMouseDownCapture":case"onMouseMove":case"onMouseMoveCapture":case"onMouseUp":case"onMouseUpCapture":case"onMouseEnter":(l=!l.disabled)||(n=n.type,l=!(n==="button"||n==="input"||n==="select"||n==="textarea")),n=!l;break e;default:n=!1}if(n)return null;if(a&&typeof a!="function")throw Error(t(231,i,typeof a));return a}var Qn=!1;if(d)try{var Si={};Object.defineProperty(Si,"passive",{get:function(){Qn=!0}}),window.addEventListener("test",Si,Si),window.removeEventListener("test",Si,Si)}catch{Qn=!1}function io(n,i,a,l,f,p,E,N,H){var re=Array.prototype.slice.call(arguments,3);try{i.apply(a,re)}catch(ve){this.onError(ve)}}var Gi=!1,Mi=null,Tr=!1,Wi=null,ro={onError:function(n){Gi=!0,Mi=n}};function so(n,i,a,l,f,p,E,N,H){Gi=!1,Mi=null,io.apply(ro,arguments)}function lu(n,i,a,l,f,p,E,N,H){if(so.apply(this,arguments),Gi){if(Gi){var re=Mi;Gi=!1,Mi=null}else throw Error(t(198));Tr||(Tr=!0,Wi=re)}}function Ei(n){var i=n,a=n;if(n.alternate)for(;i.return;)i=i.return;else{n=i;do i=n,(i.flags&4098)!==0&&(a=i.return),n=i.return;while(n)}return i.tag===3?a:null}function ao(n){if(n.tag===13){var i=n.memoizedState;if(i===null&&(n=n.alternate,n!==null&&(i=n.memoizedState)),i!==null)return i.dehydrated}return null}function C(n){if(Ei(n)!==n)throw Error(t(188))}function q(n){var i=n.alternate;if(!i){if(i=Ei(n),i===null)throw Error(t(188));return i!==n?null:n}for(var a=n,l=i;;){var f=a.return;if(f===null)break;var p=f.alternate;if(p===null){if(l=f.return,l!==null){a=l;continue}break}if(f.child===p.child){for(p=f.child;p;){if(p===a)return C(f),n;if(p===l)return C(f),i;p=p.sibling}throw Error(t(188))}if(a.return!==l.return)a=f,l=p;else{for(var E=!1,N=f.child;N;){if(N===a){E=!0,a=f,l=p;break}if(N===l){E=!0,l=f,a=p;break}N=N.sibling}if(!E){for(N=p.child;N;){if(N===a){E=!0,a=p,l=f;break}if(N===l){E=!0,l=p,a=f;break}N=N.sibling}if(!E)throw Error(t(189))}}if(a.alternate!==l)throw Error(t(190))}if(a.tag!==3)throw Error(t(188));return a.stateNode.current===a?n:i}function ie(n){return n=q(n),n!==null?ae(n):null}function ae(n){if(n.tag===5||n.tag===6)return n;for(n=n.child;n!==null;){var i=ae(n);if(i!==null)return i;n=n.sibling}return null}var Z=e.unstable_scheduleCallback,Ce=e.unstable_cancelCallback,Ue=e.unstable_shouldYield,He=e.unstable_requestPaint,Re=e.unstable_now,nt=e.unstable_getCurrentPriorityLevel,et=e.unstable_ImmediatePriority,$e=e.unstable_UserBlockingPriority,xt=e.unstable_NormalPriority,kt=e.unstable_LowPriority,It=e.unstable_IdlePriority,tn=null,ct=null;function Xe(n){if(ct&&typeof ct.onCommitFiberRoot=="function")try{ct.onCommitFiberRoot(tn,n,void 0,(n.current.flags&128)===128)}catch{}}var Ct=Math.clz32?Math.clz32:Xi,Mt=Math.log,Nn=Math.LN2;function Xi(n){return n>>>=0,n===0?32:31-(Mt(n)/Nn|0)|0}var nn=64,ji=4194304;function Nt(n){switch(n&-n){case 1:return 1;case 2:return 2;case 4:return 4;case 8:return 8;case 16:return 16;case 32:return 32;case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:return n&4194240;case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:return n&130023424;case 134217728:return 134217728;case 268435456:return 268435456;case 536870912:return 536870912;case 1073741824:return 1073741824;default:return n}}function Dn(n,i){var a=n.pendingLanes;if(a===0)return 0;var l=0,f=n.suspendedLanes,p=n.pingedLanes,E=a&268435455;if(E!==0){var N=E&~f;N!==0?l=Nt(N):(p&=E,p!==0&&(l=Nt(p)))}else E=a&~f,E!==0?l=Nt(E):p!==0&&(l=Nt(p));if(l===0)return 0;if(i!==0&&i!==l&&(i&f)===0&&(f=l&-l,p=i&-i,f>=p||f===16&&(p&4194240)!==0))return i;if((l&4)!==0&&(l|=a&16),i=n.entangledLanes,i!==0)for(n=n.entanglements,i&=l;0<i;)a=31-Ct(i),f=1<<a,l|=n[a],i&=~f;return l}function la(n,i){switch(n){case 1:case 2:case 4:return i+250;case 8:case 16:case 32:case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:return i+5e3;case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:return-1;case 134217728:case 268435456:case 536870912:case 1073741824:return-1;default:return-1}}function wn(n,i){for(var a=n.suspendedLanes,l=n.pingedLanes,f=n.expirationTimes,p=n.pendingLanes;0<p;){var E=31-Ct(p),N=1<<E,H=f[E];H===-1?((N&a)===0||(N&l)!==0)&&(f[E]=la(N,i)):H<=i&&(n.expiredLanes|=N),p&=~N}}function Ar(n){return n=n.pendingLanes&-1073741825,n!==0?n:n&1073741824?1073741824:0}function oo(){var n=nn;return nn<<=1,(nn&4194240)===0&&(nn=64),n}function is(n){for(var i=[],a=0;31>a;a++)i.push(n);return i}function ua(n,i,a){n.pendingLanes|=i,i!==536870912&&(n.suspendedLanes=0,n.pingedLanes=0),n=n.eventTimes,i=31-Ct(i),n[i]=a}function f_(n,i){var a=n.pendingLanes&~i;n.pendingLanes=i,n.suspendedLanes=0,n.pingedLanes=0,n.expiredLanes&=i,n.mutableReadLanes&=i,n.entangledLanes&=i,i=n.entanglements;var l=n.eventTimes;for(n=n.expirationTimes;0<a;){var f=31-Ct(a),p=1<<f;i[f]=0,l[f]=-1,n[f]=-1,a&=~p}}function uu(n,i){var a=n.entangledLanes|=i;for(n=n.entanglements;a;){var l=31-Ct(a),f=1<<l;f&i|n[l]&i&&(n[l]|=i),a&=~f}}var Rt=0;function Dd(n){return n&=-n,1<n?4<n?(n&268435455)!==0?16:536870912:4:1}var Id,cu,Ud,Fd,Od,fu=!1,lo=[],Yi=null,qi=null,$i=null,ca=new Map,fa=new Map,Ki=[],d_="mousedown mouseup touchcancel touchend touchstart auxclick dblclick pointercancel pointerdown pointerup dragend dragstart drop compositionend compositionstart keydown keypress keyup input textInput copy cut paste click change contextmenu reset submit".split(" ");function kd(n,i){switch(n){case"focusin":case"focusout":Yi=null;break;case"dragenter":case"dragleave":qi=null;break;case"mouseover":case"mouseout":$i=null;break;case"pointerover":case"pointerout":ca.delete(i.pointerId);break;case"gotpointercapture":case"lostpointercapture":fa.delete(i.pointerId)}}function da(n,i,a,l,f,p){return n===null||n.nativeEvent!==p?(n={blockedOn:i,domEventName:a,eventSystemFlags:l,nativeEvent:p,targetContainers:[f]},i!==null&&(i=Aa(i),i!==null&&cu(i)),n):(n.eventSystemFlags|=l,i=n.targetContainers,f!==null&&i.indexOf(f)===-1&&i.push(f),n)}function h_(n,i,a,l,f){switch(i){case"focusin":return Yi=da(Yi,n,i,a,l,f),!0;case"dragenter":return qi=da(qi,n,i,a,l,f),!0;case"mouseover":return $i=da($i,n,i,a,l,f),!0;case"pointerover":var p=f.pointerId;return ca.set(p,da(ca.get(p)||null,n,i,a,l,f)),!0;case"gotpointercapture":return p=f.pointerId,fa.set(p,da(fa.get(p)||null,n,i,a,l,f)),!0}return!1}function Bd(n){var i=Cr(n.target);if(i!==null){var a=Ei(i);if(a!==null){if(i=a.tag,i===13){if(i=ao(a),i!==null){n.blockedOn=i,Od(n.priority,function(){Ud(a)});return}}else if(i===3&&a.stateNode.current.memoizedState.isDehydrated){n.blockedOn=a.tag===3?a.stateNode.containerInfo:null;return}}}n.blockedOn=null}function uo(n){if(n.blockedOn!==null)return!1;for(var i=n.targetContainers;0<i.length;){var a=hu(n.domEventName,n.eventSystemFlags,i[0],n.nativeEvent);if(a===null){a=n.nativeEvent;var l=new a.constructor(a.type,a);X=l,a.target.dispatchEvent(l),X=null}else return i=Aa(a),i!==null&&cu(i),n.blockedOn=a,!1;i.shift()}return!0}function zd(n,i,a){uo(n)&&a.delete(i)}function p_(){fu=!1,Yi!==null&&uo(Yi)&&(Yi=null),qi!==null&&uo(qi)&&(qi=null),$i!==null&&uo($i)&&($i=null),ca.forEach(zd),fa.forEach(zd)}function ha(n,i){n.blockedOn===i&&(n.blockedOn=null,fu||(fu=!0,e.unstable_scheduleCallback(e.unstable_NormalPriority,p_)))}function pa(n){function i(f){return ha(f,n)}if(0<lo.length){ha(lo[0],n);for(var a=1;a<lo.length;a++){var l=lo[a];l.blockedOn===n&&(l.blockedOn=null)}}for(Yi!==null&&ha(Yi,n),qi!==null&&ha(qi,n),$i!==null&&ha($i,n),ca.forEach(i),fa.forEach(i),a=0;a<Ki.length;a++)l=Ki[a],l.blockedOn===n&&(l.blockedOn=null);for(;0<Ki.length&&(a=Ki[0],a.blockedOn===null);)Bd(a),a.blockedOn===null&&Ki.shift()}var rs=L.ReactCurrentBatchConfig,co=!0;function m_(n,i,a,l){var f=Rt,p=rs.transition;rs.transition=null;try{Rt=1,du(n,i,a,l)}finally{Rt=f,rs.transition=p}}function g_(n,i,a,l){var f=Rt,p=rs.transition;rs.transition=null;try{Rt=4,du(n,i,a,l)}finally{Rt=f,rs.transition=p}}function du(n,i,a,l){if(co){var f=hu(n,i,a,l);if(f===null)Pu(n,i,l,fo,a),kd(n,l);else if(h_(f,n,i,a,l))l.stopPropagation();else if(kd(n,l),i&4&&-1<d_.indexOf(n)){for(;f!==null;){var p=Aa(f);if(p!==null&&Id(p),p=hu(n,i,a,l),p===null&&Pu(n,i,l,fo,a),p===f)break;f=p}f!==null&&l.stopPropagation()}else Pu(n,i,l,null,a)}}var fo=null;function hu(n,i,a,l){if(fo=null,n=we(l),n=Cr(n),n!==null)if(i=Ei(n),i===null)n=null;else if(a=i.tag,a===13){if(n=ao(i),n!==null)return n;n=null}else if(a===3){if(i.stateNode.current.memoizedState.isDehydrated)return i.tag===3?i.stateNode.containerInfo:null;n=null}else i!==n&&(n=null);return fo=n,null}function Hd(n){switch(n){case"cancel":case"click":case"close":case"contextmenu":case"copy":case"cut":case"auxclick":case"dblclick":case"dragend":case"dragstart":case"drop":case"focusin":case"focusout":case"input":case"invalid":case"keydown":case"keypress":case"keyup":case"mousedown":case"mouseup":case"paste":case"pause":case"play":case"pointercancel":case"pointerdown":case"pointerup":case"ratechange":case"reset":case"resize":case"seeked":case"submit":case"touchcancel":case"touchend":case"touchstart":case"volumechange":case"change":case"selectionchange":case"textInput":case"compositionstart":case"compositionend":case"compositionupdate":case"beforeblur":case"afterblur":case"beforeinput":case"blur":case"fullscreenchange":case"focus":case"hashchange":case"popstate":case"select":case"selectstart":return 1;case"drag":case"dragenter":case"dragexit":case"dragleave":case"dragover":case"mousemove":case"mouseout":case"mouseover":case"pointermove":case"pointerout":case"pointerover":case"scroll":case"toggle":case"touchmove":case"wheel":case"mouseenter":case"mouseleave":case"pointerenter":case"pointerleave":return 4;case"message":switch(nt()){case et:return 1;case $e:return 4;case xt:case kt:return 16;case It:return 536870912;default:return 16}default:return 16}}var Zi=null,pu=null,ho=null;function Vd(){if(ho)return ho;var n,i=pu,a=i.length,l,f="value"in Zi?Zi.value:Zi.textContent,p=f.length;for(n=0;n<a&&i[n]===f[n];n++);var E=a-n;for(l=1;l<=E&&i[a-l]===f[p-l];l++);return ho=f.slice(n,1<l?1-l:void 0)}function po(n){var i=n.keyCode;return"charCode"in n?(n=n.charCode,n===0&&i===13&&(n=13)):n=i,n===10&&(n=13),32<=n||n===13?n:0}function mo(){return!0}function Gd(){return!1}function In(n){function i(a,l,f,p,E){this._reactName=a,this._targetInst=f,this.type=l,this.nativeEvent=p,this.target=E,this.currentTarget=null;for(var N in n)n.hasOwnProperty(N)&&(a=n[N],this[N]=a?a(p):p[N]);return this.isDefaultPrevented=(p.defaultPrevented!=null?p.defaultPrevented:p.returnValue===!1)?mo:Gd,this.isPropagationStopped=Gd,this}return le(i.prototype,{preventDefault:function(){this.defaultPrevented=!0;var a=this.nativeEvent;a&&(a.preventDefault?a.preventDefault():typeof a.returnValue!="unknown"&&(a.returnValue=!1),this.isDefaultPrevented=mo)},stopPropagation:function(){var a=this.nativeEvent;a&&(a.stopPropagation?a.stopPropagation():typeof a.cancelBubble!="unknown"&&(a.cancelBubble=!0),this.isPropagationStopped=mo)},persist:function(){},isPersistent:mo}),i}var ss={eventPhase:0,bubbles:0,cancelable:0,timeStamp:function(n){return n.timeStamp||Date.now()},defaultPrevented:0,isTrusted:0},mu=In(ss),ma=le({},ss,{view:0,detail:0}),__=In(ma),gu,_u,ga,go=le({},ma,{screenX:0,screenY:0,clientX:0,clientY:0,pageX:0,pageY:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,getModifierState:xu,button:0,buttons:0,relatedTarget:function(n){return n.relatedTarget===void 0?n.fromElement===n.srcElement?n.toElement:n.fromElement:n.relatedTarget},movementX:function(n){return"movementX"in n?n.movementX:(n!==ga&&(ga&&n.type==="mousemove"?(gu=n.screenX-ga.screenX,_u=n.screenY-ga.screenY):_u=gu=0,ga=n),gu)},movementY:function(n){return"movementY"in n?n.movementY:_u}}),Wd=In(go),v_=le({},go,{dataTransfer:0}),x_=In(v_),y_=le({},ma,{relatedTarget:0}),vu=In(y_),S_=le({},ss,{animationName:0,elapsedTime:0,pseudoElement:0}),M_=In(S_),E_=le({},ss,{clipboardData:function(n){return"clipboardData"in n?n.clipboardData:window.clipboardData}}),w_=In(E_),T_=le({},ss,{data:0}),Xd=In(T_),A_={Esc:"Escape",Spacebar:" ",Left:"ArrowLeft",Up:"ArrowUp",Right:"ArrowRight",Down:"ArrowDown",Del:"Delete",Win:"OS",Menu:"ContextMenu",Apps:"ContextMenu",Scroll:"ScrollLock",MozPrintableKey:"Unidentified"},C_={8:"Backspace",9:"Tab",12:"Clear",13:"Enter",16:"Shift",17:"Control",18:"Alt",19:"Pause",20:"CapsLock",27:"Escape",32:" ",33:"PageUp",34:"PageDown",35:"End",36:"Home",37:"ArrowLeft",38:"ArrowUp",39:"ArrowRight",40:"ArrowDown",45:"Insert",46:"Delete",112:"F1",113:"F2",114:"F3",115:"F4",116:"F5",117:"F6",118:"F7",119:"F8",120:"F9",121:"F10",122:"F11",123:"F12",144:"NumLock",145:"ScrollLock",224:"Meta"},R_={Alt:"altKey",Control:"ctrlKey",Meta:"metaKey",Shift:"shiftKey"};function b_(n){var i=this.nativeEvent;return i.getModifierState?i.getModifierState(n):(n=R_[n])?!!i[n]:!1}function xu(){return b_}var P_=le({},ma,{key:function(n){if(n.key){var i=A_[n.key]||n.key;if(i!=="Unidentified")return i}return n.type==="keypress"?(n=po(n),n===13?"Enter":String.fromCharCode(n)):n.type==="keydown"||n.type==="keyup"?C_[n.keyCode]||"Unidentified":""},code:0,location:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,repeat:0,locale:0,getModifierState:xu,charCode:function(n){return n.type==="keypress"?po(n):0},keyCode:function(n){return n.type==="keydown"||n.type==="keyup"?n.keyCode:0},which:function(n){return n.type==="keypress"?po(n):n.type==="keydown"||n.type==="keyup"?n.keyCode:0}}),L_=In(P_),N_=le({},go,{pointerId:0,width:0,height:0,pressure:0,tangentialPressure:0,tiltX:0,tiltY:0,twist:0,pointerType:0,isPrimary:0}),jd=In(N_),D_=le({},ma,{touches:0,targetTouches:0,changedTouches:0,altKey:0,metaKey:0,ctrlKey:0,shiftKey:0,getModifierState:xu}),I_=In(D_),U_=le({},ss,{propertyName:0,elapsedTime:0,pseudoElement:0}),F_=In(U_),O_=le({},go,{deltaX:function(n){return"deltaX"in n?n.deltaX:"wheelDeltaX"in n?-n.wheelDeltaX:0},deltaY:function(n){return"deltaY"in n?n.deltaY:"wheelDeltaY"in n?-n.wheelDeltaY:"wheelDelta"in n?-n.wheelDelta:0},deltaZ:0,deltaMode:0}),k_=In(O_),B_=[9,13,27,32],yu=d&&"CompositionEvent"in window,_a=null;d&&"documentMode"in document&&(_a=document.documentMode);var z_=d&&"TextEvent"in window&&!_a,Yd=d&&(!yu||_a&&8<_a&&11>=_a),qd=" ",$d=!1;function Kd(n,i){switch(n){case"keyup":return B_.indexOf(i.keyCode)!==-1;case"keydown":return i.keyCode!==229;case"keypress":case"mousedown":case"focusout":return!0;default:return!1}}function Zd(n){return n=n.detail,typeof n=="object"&&"data"in n?n.data:null}var as=!1;function H_(n,i){switch(n){case"compositionend":return Zd(i);case"keypress":return i.which!==32?null:($d=!0,qd);case"textInput":return n=i.data,n===qd&&$d?null:n;default:return null}}function V_(n,i){if(as)return n==="compositionend"||!yu&&Kd(n,i)?(n=Vd(),ho=pu=Zi=null,as=!1,n):null;switch(n){case"paste":return null;case"keypress":if(!(i.ctrlKey||i.altKey||i.metaKey)||i.ctrlKey&&i.altKey){if(i.char&&1<i.char.length)return i.char;if(i.which)return String.fromCharCode(i.which)}return null;case"compositionend":return Yd&&i.locale!=="ko"?null:i.data;default:return null}}var G_={color:!0,date:!0,datetime:!0,"datetime-local":!0,email:!0,month:!0,number:!0,password:!0,range:!0,search:!0,tel:!0,text:!0,time:!0,url:!0,week:!0};function Qd(n){var i=n&&n.nodeName&&n.nodeName.toLowerCase();return i==="input"?!!G_[n.type]:i==="textarea"}function Jd(n,i,a,l){pe(l),i=So(i,"onChange"),0<i.length&&(a=new mu("onChange","change",null,a,l),n.push({event:a,listeners:i}))}var va=null,xa=null;function W_(n){_h(n,0)}function _o(n){var i=fs(n);if(z(i))return n}function X_(n,i){if(n==="change")return i}var eh=!1;if(d){var Su;if(d){var Mu="oninput"in document;if(!Mu){var th=document.createElement("div");th.setAttribute("oninput","return;"),Mu=typeof th.oninput=="function"}Su=Mu}else Su=!1;eh=Su&&(!document.documentMode||9<document.documentMode)}function nh(){va&&(va.detachEvent("onpropertychange",ih),xa=va=null)}function ih(n){if(n.propertyName==="value"&&_o(xa)){var i=[];Jd(i,xa,n,we(n)),qt(W_,i)}}function j_(n,i,a){n==="focusin"?(nh(),va=i,xa=a,va.attachEvent("onpropertychange",ih)):n==="focusout"&&nh()}function Y_(n){if(n==="selectionchange"||n==="keyup"||n==="keydown")return _o(xa)}function q_(n,i){if(n==="click")return _o(i)}function $_(n,i){if(n==="input"||n==="change")return _o(i)}function K_(n,i){return n===i&&(n!==0||1/n===1/i)||n!==n&&i!==i}var Jn=typeof Object.is=="function"?Object.is:K_;function ya(n,i){if(Jn(n,i))return!0;if(typeof n!="object"||n===null||typeof i!="object"||i===null)return!1;var a=Object.keys(n),l=Object.keys(i);if(a.length!==l.length)return!1;for(l=0;l<a.length;l++){var f=a[l];if(!h.call(i,f)||!Jn(n[f],i[f]))return!1}return!0}function rh(n){for(;n&&n.firstChild;)n=n.firstChild;return n}function sh(n,i){var a=rh(n);n=0;for(var l;a;){if(a.nodeType===3){if(l=n+a.textContent.length,n<=i&&l>=i)return{node:a,offset:i-n};n=l}e:{for(;a;){if(a.nextSibling){a=a.nextSibling;break e}a=a.parentNode}a=void 0}a=rh(a)}}function ah(n,i){return n&&i?n===i?!0:n&&n.nodeType===3?!1:i&&i.nodeType===3?ah(n,i.parentNode):"contains"in n?n.contains(i):n.compareDocumentPosition?!!(n.compareDocumentPosition(i)&16):!1:!1}function oh(){for(var n=window,i=bt();i instanceof n.HTMLIFrameElement;){try{var a=typeof i.contentWindow.location.href=="string"}catch{a=!1}if(a)n=i.contentWindow;else break;i=bt(n.document)}return i}function Eu(n){var i=n&&n.nodeName&&n.nodeName.toLowerCase();return i&&(i==="input"&&(n.type==="text"||n.type==="search"||n.type==="tel"||n.type==="url"||n.type==="password")||i==="textarea"||n.contentEditable==="true")}function Z_(n){var i=oh(),a=n.focusedElem,l=n.selectionRange;if(i!==a&&a&&a.ownerDocument&&ah(a.ownerDocument.documentElement,a)){if(l!==null&&Eu(a)){if(i=l.start,n=l.end,n===void 0&&(n=i),"selectionStart"in a)a.selectionStart=i,a.selectionEnd=Math.min(n,a.value.length);else if(n=(i=a.ownerDocument||document)&&i.defaultView||window,n.getSelection){n=n.getSelection();var f=a.textContent.length,p=Math.min(l.start,f);l=l.end===void 0?p:Math.min(l.end,f),!n.extend&&p>l&&(f=l,l=p,p=f),f=sh(a,p);var E=sh(a,l);f&&E&&(n.rangeCount!==1||n.anchorNode!==f.node||n.anchorOffset!==f.offset||n.focusNode!==E.node||n.focusOffset!==E.offset)&&(i=i.createRange(),i.setStart(f.node,f.offset),n.removeAllRanges(),p>l?(n.addRange(i),n.extend(E.node,E.offset)):(i.setEnd(E.node,E.offset),n.addRange(i)))}}for(i=[],n=a;n=n.parentNode;)n.nodeType===1&&i.push({element:n,left:n.scrollLeft,top:n.scrollTop});for(typeof a.focus=="function"&&a.focus(),a=0;a<i.length;a++)n=i[a],n.element.scrollLeft=n.left,n.element.scrollTop=n.top}}var Q_=d&&"documentMode"in document&&11>=document.documentMode,os=null,wu=null,Sa=null,Tu=!1;function lh(n,i,a){var l=a.window===a?a.document:a.nodeType===9?a:a.ownerDocument;Tu||os==null||os!==bt(l)||(l=os,"selectionStart"in l&&Eu(l)?l={start:l.selectionStart,end:l.selectionEnd}:(l=(l.ownerDocument&&l.ownerDocument.defaultView||window).getSelection(),l={anchorNode:l.anchorNode,anchorOffset:l.anchorOffset,focusNode:l.focusNode,focusOffset:l.focusOffset}),Sa&&ya(Sa,l)||(Sa=l,l=So(wu,"onSelect"),0<l.length&&(i=new mu("onSelect","select",null,i,a),n.push({event:i,listeners:l}),i.target=os)))}function vo(n,i){var a={};return a[n.toLowerCase()]=i.toLowerCase(),a["Webkit"+n]="webkit"+i,a["Moz"+n]="moz"+i,a}var ls={animationend:vo("Animation","AnimationEnd"),animationiteration:vo("Animation","AnimationIteration"),animationstart:vo("Animation","AnimationStart"),transitionend:vo("Transition","TransitionEnd")},Au={},uh={};d&&(uh=document.createElement("div").style,"AnimationEvent"in window||(delete ls.animationend.animation,delete ls.animationiteration.animation,delete ls.animationstart.animation),"TransitionEvent"in window||delete ls.transitionend.transition);function xo(n){if(Au[n])return Au[n];if(!ls[n])return n;var i=ls[n],a;for(a in i)if(i.hasOwnProperty(a)&&a in uh)return Au[n]=i[a];return n}var ch=xo("animationend"),fh=xo("animationiteration"),dh=xo("animationstart"),hh=xo("transitionend"),ph=new Map,mh="abort auxClick cancel canPlay canPlayThrough click close contextMenu copy cut drag dragEnd dragEnter dragExit dragLeave dragOver dragStart drop durationChange emptied encrypted ended error gotPointerCapture input invalid keyDown keyPress keyUp load loadedData loadedMetadata loadStart lostPointerCapture mouseDown mouseMove mouseOut mouseOver mouseUp paste pause play playing pointerCancel pointerDown pointerMove pointerOut pointerOver pointerUp progress rateChange reset resize seeked seeking stalled submit suspend timeUpdate touchCancel touchEnd touchStart volumeChange scroll toggle touchMove waiting wheel".split(" ");function Qi(n,i){ph.set(n,i),u(i,[n])}for(var Cu=0;Cu<mh.length;Cu++){var Ru=mh[Cu],J_=Ru.toLowerCase(),ev=Ru[0].toUpperCase()+Ru.slice(1);Qi(J_,"on"+ev)}Qi(ch,"onAnimationEnd"),Qi(fh,"onAnimationIteration"),Qi(dh,"onAnimationStart"),Qi("dblclick","onDoubleClick"),Qi("focusin","onFocus"),Qi("focusout","onBlur"),Qi(hh,"onTransitionEnd"),c("onMouseEnter",["mouseout","mouseover"]),c("onMouseLeave",["mouseout","mouseover"]),c("onPointerEnter",["pointerout","pointerover"]),c("onPointerLeave",["pointerout","pointerover"]),u("onChange","change click focusin focusout input keydown keyup selectionchange".split(" ")),u("onSelect","focusout contextmenu dragend focusin keydown keyup mousedown mouseup selectionchange".split(" ")),u("onBeforeInput",["compositionend","keypress","textInput","paste"]),u("onCompositionEnd","compositionend focusout keydown keypress keyup mousedown".split(" ")),u("onCompositionStart","compositionstart focusout keydown keypress keyup mousedown".split(" ")),u("onCompositionUpdate","compositionupdate focusout keydown keypress keyup mousedown".split(" "));var Ma="abort canplay canplaythrough durationchange emptied encrypted ended error loadeddata loadedmetadata loadstart pause play playing progress ratechange resize seeked seeking stalled suspend timeupdate volumechange waiting".split(" "),tv=new Set("cancel close invalid load scroll toggle".split(" ").concat(Ma));function gh(n,i,a){var l=n.type||"unknown-event";n.currentTarget=a,lu(l,i,void 0,n),n.currentTarget=null}function _h(n,i){i=(i&4)!==0;for(var a=0;a<n.length;a++){var l=n[a],f=l.event;l=l.listeners;e:{var p=void 0;if(i)for(var E=l.length-1;0<=E;E--){var N=l[E],H=N.instance,re=N.currentTarget;if(N=N.listener,H!==p&&f.isPropagationStopped())break e;gh(f,N,re),p=H}else for(E=0;E<l.length;E++){if(N=l[E],H=N.instance,re=N.currentTarget,N=N.listener,H!==p&&f.isPropagationStopped())break e;gh(f,N,re),p=H}}}if(Tr)throw n=Wi,Tr=!1,Wi=null,n}function Ut(n,i){var a=i[Fu];a===void 0&&(a=i[Fu]=new Set);var l=n+"__bubble";a.has(l)||(vh(i,n,2,!1),a.add(l))}function bu(n,i,a){var l=0;i&&(l|=4),vh(a,n,l,i)}var yo="_reactListening"+Math.random().toString(36).slice(2);function Ea(n){if(!n[yo]){n[yo]=!0,r.forEach(function(a){a!=="selectionchange"&&(tv.has(a)||bu(a,!1,n),bu(a,!0,n))});var i=n.nodeType===9?n:n.ownerDocument;i===null||i[yo]||(i[yo]=!0,bu("selectionchange",!1,i))}}function vh(n,i,a,l){switch(Hd(i)){case 1:var f=m_;break;case 4:f=g_;break;default:f=du}a=f.bind(null,i,a,n),f=void 0,!Qn||i!=="touchstart"&&i!=="touchmove"&&i!=="wheel"||(f=!0),l?f!==void 0?n.addEventListener(i,a,{capture:!0,passive:f}):n.addEventListener(i,a,!0):f!==void 0?n.addEventListener(i,a,{passive:f}):n.addEventListener(i,a,!1)}function Pu(n,i,a,l,f){var p=l;if((i&1)===0&&(i&2)===0&&l!==null)e:for(;;){if(l===null)return;var E=l.tag;if(E===3||E===4){var N=l.stateNode.containerInfo;if(N===f||N.nodeType===8&&N.parentNode===f)break;if(E===4)for(E=l.return;E!==null;){var H=E.tag;if((H===3||H===4)&&(H=E.stateNode.containerInfo,H===f||H.nodeType===8&&H.parentNode===f))return;E=E.return}for(;N!==null;){if(E=Cr(N),E===null)return;if(H=E.tag,H===5||H===6){l=p=E;continue e}N=N.parentNode}}l=l.return}qt(function(){var re=p,ve=we(a),Se=[];e:{var ge=ph.get(n);if(ge!==void 0){var Fe=mu,ze=n;switch(n){case"keypress":if(po(a)===0)break e;case"keydown":case"keyup":Fe=L_;break;case"focusin":ze="focus",Fe=vu;break;case"focusout":ze="blur",Fe=vu;break;case"beforeblur":case"afterblur":Fe=vu;break;case"click":if(a.button===2)break e;case"auxclick":case"dblclick":case"mousedown":case"mousemove":case"mouseup":case"mouseout":case"mouseover":case"contextmenu":Fe=Wd;break;case"drag":case"dragend":case"dragenter":case"dragexit":case"dragleave":case"dragover":case"dragstart":case"drop":Fe=x_;break;case"touchcancel":case"touchend":case"touchmove":case"touchstart":Fe=I_;break;case ch:case fh:case dh:Fe=M_;break;case hh:Fe=F_;break;case"scroll":Fe=__;break;case"wheel":Fe=k_;break;case"copy":case"cut":case"paste":Fe=w_;break;case"gotpointercapture":case"lostpointercapture":case"pointercancel":case"pointerdown":case"pointermove":case"pointerout":case"pointerover":case"pointerup":Fe=jd}var Ve=(i&4)!==0,Wt=!Ve&&n==="scroll",Q=Ve?ge!==null?ge+"Capture":null:ge;Ve=[];for(var W=re,ee;W!==null;){ee=W;var Ae=ee.stateNode;if(ee.tag===5&&Ae!==null&&(ee=Ae,Q!==null&&(Ae=Vi(W,Q),Ae!=null&&Ve.push(wa(W,Ae,ee)))),Wt)break;W=W.return}0<Ve.length&&(ge=new Fe(ge,ze,null,a,ve),Se.push({event:ge,listeners:Ve}))}}if((i&7)===0){e:{if(ge=n==="mouseover"||n==="pointerover",Fe=n==="mouseout"||n==="pointerout",ge&&a!==X&&(ze=a.relatedTarget||a.fromElement)&&(Cr(ze)||ze[wi]))break e;if((Fe||ge)&&(ge=ve.window===ve?ve:(ge=ve.ownerDocument)?ge.defaultView||ge.parentWindow:window,Fe?(ze=a.relatedTarget||a.toElement,Fe=re,ze=ze?Cr(ze):null,ze!==null&&(Wt=Ei(ze),ze!==Wt||ze.tag!==5&&ze.tag!==6)&&(ze=null)):(Fe=null,ze=re),Fe!==ze)){if(Ve=Wd,Ae="onMouseLeave",Q="onMouseEnter",W="mouse",(n==="pointerout"||n==="pointerover")&&(Ve=jd,Ae="onPointerLeave",Q="onPointerEnter",W="pointer"),Wt=Fe==null?ge:fs(Fe),ee=ze==null?ge:fs(ze),ge=new Ve(Ae,W+"leave",Fe,a,ve),ge.target=Wt,ge.relatedTarget=ee,Ae=null,Cr(ve)===re&&(Ve=new Ve(Q,W+"enter",ze,a,ve),Ve.target=ee,Ve.relatedTarget=Wt,Ae=Ve),Wt=Ae,Fe&&ze)t:{for(Ve=Fe,Q=ze,W=0,ee=Ve;ee;ee=us(ee))W++;for(ee=0,Ae=Q;Ae;Ae=us(Ae))ee++;for(;0<W-ee;)Ve=us(Ve),W--;for(;0<ee-W;)Q=us(Q),ee--;for(;W--;){if(Ve===Q||Q!==null&&Ve===Q.alternate)break t;Ve=us(Ve),Q=us(Q)}Ve=null}else Ve=null;Fe!==null&&xh(Se,ge,Fe,Ve,!1),ze!==null&&Wt!==null&&xh(Se,Wt,ze,Ve,!0)}}e:{if(ge=re?fs(re):window,Fe=ge.nodeName&&ge.nodeName.toLowerCase(),Fe==="select"||Fe==="input"&&ge.type==="file")var Ge=X_;else if(Qd(ge))if(eh)Ge=$_;else{Ge=Y_;var Ze=j_}else(Fe=ge.nodeName)&&Fe.toLowerCase()==="input"&&(ge.type==="checkbox"||ge.type==="radio")&&(Ge=q_);if(Ge&&(Ge=Ge(n,re))){Jd(Se,Ge,a,ve);break e}Ze&&Ze(n,ge,re),n==="focusout"&&(Ze=ge._wrapperState)&&Ze.controlled&&ge.type==="number"&&rt(ge,"number",ge.value)}switch(Ze=re?fs(re):window,n){case"focusin":(Qd(Ze)||Ze.contentEditable==="true")&&(os=Ze,wu=re,Sa=null);break;case"focusout":Sa=wu=os=null;break;case"mousedown":Tu=!0;break;case"contextmenu":case"mouseup":case"dragend":Tu=!1,lh(Se,a,ve);break;case"selectionchange":if(Q_)break;case"keydown":case"keyup":lh(Se,a,ve)}var Qe;if(yu)e:{switch(n){case"compositionstart":var at="onCompositionStart";break e;case"compositionend":at="onCompositionEnd";break e;case"compositionupdate":at="onCompositionUpdate";break e}at=void 0}else as?Kd(n,a)&&(at="onCompositionEnd"):n==="keydown"&&a.keyCode===229&&(at="onCompositionStart");at&&(Yd&&a.locale!=="ko"&&(as||at!=="onCompositionStart"?at==="onCompositionEnd"&&as&&(Qe=Vd()):(Zi=ve,pu="value"in Zi?Zi.value:Zi.textContent,as=!0)),Ze=So(re,at),0<Ze.length&&(at=new Xd(at,n,null,a,ve),Se.push({event:at,listeners:Ze}),Qe?at.data=Qe:(Qe=Zd(a),Qe!==null&&(at.data=Qe)))),(Qe=z_?H_(n,a):V_(n,a))&&(re=So(re,"onBeforeInput"),0<re.length&&(ve=new Xd("onBeforeInput","beforeinput",null,a,ve),Se.push({event:ve,listeners:re}),ve.data=Qe))}_h(Se,i)})}function wa(n,i,a){return{instance:n,listener:i,currentTarget:a}}function So(n,i){for(var a=i+"Capture",l=[];n!==null;){var f=n,p=f.stateNode;f.tag===5&&p!==null&&(f=p,p=Vi(n,a),p!=null&&l.unshift(wa(n,p,f)),p=Vi(n,i),p!=null&&l.push(wa(n,p,f))),n=n.return}return l}function us(n){if(n===null)return null;do n=n.return;while(n&&n.tag!==5);return n||null}function xh(n,i,a,l,f){for(var p=i._reactName,E=[];a!==null&&a!==l;){var N=a,H=N.alternate,re=N.stateNode;if(H!==null&&H===l)break;N.tag===5&&re!==null&&(N=re,f?(H=Vi(a,p),H!=null&&E.unshift(wa(a,H,N))):f||(H=Vi(a,p),H!=null&&E.push(wa(a,H,N)))),a=a.return}E.length!==0&&n.push({event:i,listeners:E})}var nv=/\r\n?/g,iv=/\u0000|\uFFFD/g;function yh(n){return(typeof n=="string"?n:""+n).replace(nv,`
`).replace(iv,"")}function Mo(n,i,a){if(i=yh(i),yh(n)!==i&&a)throw Error(t(425))}function Eo(){}var Lu=null,Nu=null;function Du(n,i){return n==="textarea"||n==="noscript"||typeof i.children=="string"||typeof i.children=="number"||typeof i.dangerouslySetInnerHTML=="object"&&i.dangerouslySetInnerHTML!==null&&i.dangerouslySetInnerHTML.__html!=null}var Iu=typeof setTimeout=="function"?setTimeout:void 0,rv=typeof clearTimeout=="function"?clearTimeout:void 0,Sh=typeof Promise=="function"?Promise:void 0,sv=typeof queueMicrotask=="function"?queueMicrotask:typeof Sh<"u"?function(n){return Sh.resolve(null).then(n).catch(av)}:Iu;function av(n){setTimeout(function(){throw n})}function Uu(n,i){var a=i,l=0;do{var f=a.nextSibling;if(n.removeChild(a),f&&f.nodeType===8)if(a=f.data,a==="/$"){if(l===0){n.removeChild(f),pa(i);return}l--}else a!=="$"&&a!=="$?"&&a!=="$!"||l++;a=f}while(a);pa(i)}function Ji(n){for(;n!=null;n=n.nextSibling){var i=n.nodeType;if(i===1||i===3)break;if(i===8){if(i=n.data,i==="$"||i==="$!"||i==="$?")break;if(i==="/$")return null}}return n}function Mh(n){n=n.previousSibling;for(var i=0;n;){if(n.nodeType===8){var a=n.data;if(a==="$"||a==="$!"||a==="$?"){if(i===0)return n;i--}else a==="/$"&&i++}n=n.previousSibling}return null}var cs=Math.random().toString(36).slice(2),hi="__reactFiber$"+cs,Ta="__reactProps$"+cs,wi="__reactContainer$"+cs,Fu="__reactEvents$"+cs,ov="__reactListeners$"+cs,lv="__reactHandles$"+cs;function Cr(n){var i=n[hi];if(i)return i;for(var a=n.parentNode;a;){if(i=a[wi]||a[hi]){if(a=i.alternate,i.child!==null||a!==null&&a.child!==null)for(n=Mh(n);n!==null;){if(a=n[hi])return a;n=Mh(n)}return i}n=a,a=n.parentNode}return null}function Aa(n){return n=n[hi]||n[wi],!n||n.tag!==5&&n.tag!==6&&n.tag!==13&&n.tag!==3?null:n}function fs(n){if(n.tag===5||n.tag===6)return n.stateNode;throw Error(t(33))}function wo(n){return n[Ta]||null}var Ou=[],ds=-1;function er(n){return{current:n}}function Ft(n){0>ds||(n.current=Ou[ds],Ou[ds]=null,ds--)}function Dt(n,i){ds++,Ou[ds]=n.current,n.current=i}var tr={},cn=er(tr),Tn=er(!1),Rr=tr;function hs(n,i){var a=n.type.contextTypes;if(!a)return tr;var l=n.stateNode;if(l&&l.__reactInternalMemoizedUnmaskedChildContext===i)return l.__reactInternalMemoizedMaskedChildContext;var f={},p;for(p in a)f[p]=i[p];return l&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=i,n.__reactInternalMemoizedMaskedChildContext=f),f}function An(n){return n=n.childContextTypes,n!=null}function To(){Ft(Tn),Ft(cn)}function Eh(n,i,a){if(cn.current!==tr)throw Error(t(168));Dt(cn,i),Dt(Tn,a)}function wh(n,i,a){var l=n.stateNode;if(i=i.childContextTypes,typeof l.getChildContext!="function")return a;l=l.getChildContext();for(var f in l)if(!(f in i))throw Error(t(108,Me(n)||"Unknown",f));return le({},a,l)}function Ao(n){return n=(n=n.stateNode)&&n.__reactInternalMemoizedMergedChildContext||tr,Rr=cn.current,Dt(cn,n),Dt(Tn,Tn.current),!0}function Th(n,i,a){var l=n.stateNode;if(!l)throw Error(t(169));a?(n=wh(n,i,Rr),l.__reactInternalMemoizedMergedChildContext=n,Ft(Tn),Ft(cn),Dt(cn,n)):Ft(Tn),Dt(Tn,a)}var Ti=null,Co=!1,ku=!1;function Ah(n){Ti===null?Ti=[n]:Ti.push(n)}function uv(n){Co=!0,Ah(n)}function nr(){if(!ku&&Ti!==null){ku=!0;var n=0,i=Rt;try{var a=Ti;for(Rt=1;n<a.length;n++){var l=a[n];do l=l(!0);while(l!==null)}Ti=null,Co=!1}catch(f){throw Ti!==null&&(Ti=Ti.slice(n+1)),Z(et,nr),f}finally{Rt=i,ku=!1}}return null}var ps=[],ms=0,Ro=null,bo=0,Vn=[],Gn=0,br=null,Ai=1,Ci="";function Pr(n,i){ps[ms++]=bo,ps[ms++]=Ro,Ro=n,bo=i}function Ch(n,i,a){Vn[Gn++]=Ai,Vn[Gn++]=Ci,Vn[Gn++]=br,br=n;var l=Ai;n=Ci;var f=32-Ct(l)-1;l&=~(1<<f),a+=1;var p=32-Ct(i)+f;if(30<p){var E=f-f%5;p=(l&(1<<E)-1).toString(32),l>>=E,f-=E,Ai=1<<32-Ct(i)+f|a<<f|l,Ci=p+n}else Ai=1<<p|a<<f|l,Ci=n}function Bu(n){n.return!==null&&(Pr(n,1),Ch(n,1,0))}function zu(n){for(;n===Ro;)Ro=ps[--ms],ps[ms]=null,bo=ps[--ms],ps[ms]=null;for(;n===br;)br=Vn[--Gn],Vn[Gn]=null,Ci=Vn[--Gn],Vn[Gn]=null,Ai=Vn[--Gn],Vn[Gn]=null}var Un=null,Fn=null,Bt=!1,ei=null;function Rh(n,i){var a=Yn(5,null,null,0);a.elementType="DELETED",a.stateNode=i,a.return=n,i=n.deletions,i===null?(n.deletions=[a],n.flags|=16):i.push(a)}function bh(n,i){switch(n.tag){case 5:var a=n.type;return i=i.nodeType!==1||a.toLowerCase()!==i.nodeName.toLowerCase()?null:i,i!==null?(n.stateNode=i,Un=n,Fn=Ji(i.firstChild),!0):!1;case 6:return i=n.pendingProps===""||i.nodeType!==3?null:i,i!==null?(n.stateNode=i,Un=n,Fn=null,!0):!1;case 13:return i=i.nodeType!==8?null:i,i!==null?(a=br!==null?{id:Ai,overflow:Ci}:null,n.memoizedState={dehydrated:i,treeContext:a,retryLane:1073741824},a=Yn(18,null,null,0),a.stateNode=i,a.return=n,n.child=a,Un=n,Fn=null,!0):!1;default:return!1}}function Hu(n){return(n.mode&1)!==0&&(n.flags&128)===0}function Vu(n){if(Bt){var i=Fn;if(i){var a=i;if(!bh(n,i)){if(Hu(n))throw Error(t(418));i=Ji(a.nextSibling);var l=Un;i&&bh(n,i)?Rh(l,a):(n.flags=n.flags&-4097|2,Bt=!1,Un=n)}}else{if(Hu(n))throw Error(t(418));n.flags=n.flags&-4097|2,Bt=!1,Un=n}}}function Ph(n){for(n=n.return;n!==null&&n.tag!==5&&n.tag!==3&&n.tag!==13;)n=n.return;Un=n}function Po(n){if(n!==Un)return!1;if(!Bt)return Ph(n),Bt=!0,!1;var i;if((i=n.tag!==3)&&!(i=n.tag!==5)&&(i=n.type,i=i!=="head"&&i!=="body"&&!Du(n.type,n.memoizedProps)),i&&(i=Fn)){if(Hu(n))throw Lh(),Error(t(418));for(;i;)Rh(n,i),i=Ji(i.nextSibling)}if(Ph(n),n.tag===13){if(n=n.memoizedState,n=n!==null?n.dehydrated:null,!n)throw Error(t(317));e:{for(n=n.nextSibling,i=0;n;){if(n.nodeType===8){var a=n.data;if(a==="/$"){if(i===0){Fn=Ji(n.nextSibling);break e}i--}else a!=="$"&&a!=="$!"&&a!=="$?"||i++}n=n.nextSibling}Fn=null}}else Fn=Un?Ji(n.stateNode.nextSibling):null;return!0}function Lh(){for(var n=Fn;n;)n=Ji(n.nextSibling)}function gs(){Fn=Un=null,Bt=!1}function Gu(n){ei===null?ei=[n]:ei.push(n)}var cv=L.ReactCurrentBatchConfig;function Ca(n,i,a){if(n=a.ref,n!==null&&typeof n!="function"&&typeof n!="object"){if(a._owner){if(a=a._owner,a){if(a.tag!==1)throw Error(t(309));var l=a.stateNode}if(!l)throw Error(t(147,n));var f=l,p=""+n;return i!==null&&i.ref!==null&&typeof i.ref=="function"&&i.ref._stringRef===p?i.ref:(i=function(E){var N=f.refs;E===null?delete N[p]:N[p]=E},i._stringRef=p,i)}if(typeof n!="string")throw Error(t(284));if(!a._owner)throw Error(t(290,n))}return n}function Lo(n,i){throw n=Object.prototype.toString.call(i),Error(t(31,n==="[object Object]"?"object with keys {"+Object.keys(i).join(", ")+"}":n))}function Nh(n){var i=n._init;return i(n._payload)}function Dh(n){function i(Q,W){if(n){var ee=Q.deletions;ee===null?(Q.deletions=[W],Q.flags|=16):ee.push(W)}}function a(Q,W){if(!n)return null;for(;W!==null;)i(Q,W),W=W.sibling;return null}function l(Q,W){for(Q=new Map;W!==null;)W.key!==null?Q.set(W.key,W):Q.set(W.index,W),W=W.sibling;return Q}function f(Q,W){return Q=cr(Q,W),Q.index=0,Q.sibling=null,Q}function p(Q,W,ee){return Q.index=ee,n?(ee=Q.alternate,ee!==null?(ee=ee.index,ee<W?(Q.flags|=2,W):ee):(Q.flags|=2,W)):(Q.flags|=1048576,W)}function E(Q){return n&&Q.alternate===null&&(Q.flags|=2),Q}function N(Q,W,ee,Ae){return W===null||W.tag!==6?(W=Ic(ee,Q.mode,Ae),W.return=Q,W):(W=f(W,ee),W.return=Q,W)}function H(Q,W,ee,Ae){var Ge=ee.type;return Ge===D?ve(Q,W,ee.props.children,Ae,ee.key):W!==null&&(W.elementType===Ge||typeof Ge=="object"&&Ge!==null&&Ge.$$typeof===B&&Nh(Ge)===W.type)?(Ae=f(W,ee.props),Ae.ref=Ca(Q,W,ee),Ae.return=Q,Ae):(Ae=tl(ee.type,ee.key,ee.props,null,Q.mode,Ae),Ae.ref=Ca(Q,W,ee),Ae.return=Q,Ae)}function re(Q,W,ee,Ae){return W===null||W.tag!==4||W.stateNode.containerInfo!==ee.containerInfo||W.stateNode.implementation!==ee.implementation?(W=Uc(ee,Q.mode,Ae),W.return=Q,W):(W=f(W,ee.children||[]),W.return=Q,W)}function ve(Q,W,ee,Ae,Ge){return W===null||W.tag!==7?(W=kr(ee,Q.mode,Ae,Ge),W.return=Q,W):(W=f(W,ee),W.return=Q,W)}function Se(Q,W,ee){if(typeof W=="string"&&W!==""||typeof W=="number")return W=Ic(""+W,Q.mode,ee),W.return=Q,W;if(typeof W=="object"&&W!==null){switch(W.$$typeof){case $:return ee=tl(W.type,W.key,W.props,null,Q.mode,ee),ee.ref=Ca(Q,null,W),ee.return=Q,ee;case O:return W=Uc(W,Q.mode,ee),W.return=Q,W;case B:var Ae=W._init;return Se(Q,Ae(W._payload),ee)}if(U(W)||ue(W))return W=kr(W,Q.mode,ee,null),W.return=Q,W;Lo(Q,W)}return null}function ge(Q,W,ee,Ae){var Ge=W!==null?W.key:null;if(typeof ee=="string"&&ee!==""||typeof ee=="number")return Ge!==null?null:N(Q,W,""+ee,Ae);if(typeof ee=="object"&&ee!==null){switch(ee.$$typeof){case $:return ee.key===Ge?H(Q,W,ee,Ae):null;case O:return ee.key===Ge?re(Q,W,ee,Ae):null;case B:return Ge=ee._init,ge(Q,W,Ge(ee._payload),Ae)}if(U(ee)||ue(ee))return Ge!==null?null:ve(Q,W,ee,Ae,null);Lo(Q,ee)}return null}function Fe(Q,W,ee,Ae,Ge){if(typeof Ae=="string"&&Ae!==""||typeof Ae=="number")return Q=Q.get(ee)||null,N(W,Q,""+Ae,Ge);if(typeof Ae=="object"&&Ae!==null){switch(Ae.$$typeof){case $:return Q=Q.get(Ae.key===null?ee:Ae.key)||null,H(W,Q,Ae,Ge);case O:return Q=Q.get(Ae.key===null?ee:Ae.key)||null,re(W,Q,Ae,Ge);case B:var Ze=Ae._init;return Fe(Q,W,ee,Ze(Ae._payload),Ge)}if(U(Ae)||ue(Ae))return Q=Q.get(ee)||null,ve(W,Q,Ae,Ge,null);Lo(W,Ae)}return null}function ze(Q,W,ee,Ae){for(var Ge=null,Ze=null,Qe=W,at=W=0,an=null;Qe!==null&&at<ee.length;at++){Qe.index>at?(an=Qe,Qe=null):an=Qe.sibling;var Et=ge(Q,Qe,ee[at],Ae);if(Et===null){Qe===null&&(Qe=an);break}n&&Qe&&Et.alternate===null&&i(Q,Qe),W=p(Et,W,at),Ze===null?Ge=Et:Ze.sibling=Et,Ze=Et,Qe=an}if(at===ee.length)return a(Q,Qe),Bt&&Pr(Q,at),Ge;if(Qe===null){for(;at<ee.length;at++)Qe=Se(Q,ee[at],Ae),Qe!==null&&(W=p(Qe,W,at),Ze===null?Ge=Qe:Ze.sibling=Qe,Ze=Qe);return Bt&&Pr(Q,at),Ge}for(Qe=l(Q,Qe);at<ee.length;at++)an=Fe(Qe,Q,at,ee[at],Ae),an!==null&&(n&&an.alternate!==null&&Qe.delete(an.key===null?at:an.key),W=p(an,W,at),Ze===null?Ge=an:Ze.sibling=an,Ze=an);return n&&Qe.forEach(function(fr){return i(Q,fr)}),Bt&&Pr(Q,at),Ge}function Ve(Q,W,ee,Ae){var Ge=ue(ee);if(typeof Ge!="function")throw Error(t(150));if(ee=Ge.call(ee),ee==null)throw Error(t(151));for(var Ze=Ge=null,Qe=W,at=W=0,an=null,Et=ee.next();Qe!==null&&!Et.done;at++,Et=ee.next()){Qe.index>at?(an=Qe,Qe=null):an=Qe.sibling;var fr=ge(Q,Qe,Et.value,Ae);if(fr===null){Qe===null&&(Qe=an);break}n&&Qe&&fr.alternate===null&&i(Q,Qe),W=p(fr,W,at),Ze===null?Ge=fr:Ze.sibling=fr,Ze=fr,Qe=an}if(Et.done)return a(Q,Qe),Bt&&Pr(Q,at),Ge;if(Qe===null){for(;!Et.done;at++,Et=ee.next())Et=Se(Q,Et.value,Ae),Et!==null&&(W=p(Et,W,at),Ze===null?Ge=Et:Ze.sibling=Et,Ze=Et);return Bt&&Pr(Q,at),Ge}for(Qe=l(Q,Qe);!Et.done;at++,Et=ee.next())Et=Fe(Qe,Q,at,Et.value,Ae),Et!==null&&(n&&Et.alternate!==null&&Qe.delete(Et.key===null?at:Et.key),W=p(Et,W,at),Ze===null?Ge=Et:Ze.sibling=Et,Ze=Et);return n&&Qe.forEach(function(Gv){return i(Q,Gv)}),Bt&&Pr(Q,at),Ge}function Wt(Q,W,ee,Ae){if(typeof ee=="object"&&ee!==null&&ee.type===D&&ee.key===null&&(ee=ee.props.children),typeof ee=="object"&&ee!==null){switch(ee.$$typeof){case $:e:{for(var Ge=ee.key,Ze=W;Ze!==null;){if(Ze.key===Ge){if(Ge=ee.type,Ge===D){if(Ze.tag===7){a(Q,Ze.sibling),W=f(Ze,ee.props.children),W.return=Q,Q=W;break e}}else if(Ze.elementType===Ge||typeof Ge=="object"&&Ge!==null&&Ge.$$typeof===B&&Nh(Ge)===Ze.type){a(Q,Ze.sibling),W=f(Ze,ee.props),W.ref=Ca(Q,Ze,ee),W.return=Q,Q=W;break e}a(Q,Ze);break}else i(Q,Ze);Ze=Ze.sibling}ee.type===D?(W=kr(ee.props.children,Q.mode,Ae,ee.key),W.return=Q,Q=W):(Ae=tl(ee.type,ee.key,ee.props,null,Q.mode,Ae),Ae.ref=Ca(Q,W,ee),Ae.return=Q,Q=Ae)}return E(Q);case O:e:{for(Ze=ee.key;W!==null;){if(W.key===Ze)if(W.tag===4&&W.stateNode.containerInfo===ee.containerInfo&&W.stateNode.implementation===ee.implementation){a(Q,W.sibling),W=f(W,ee.children||[]),W.return=Q,Q=W;break e}else{a(Q,W);break}else i(Q,W);W=W.sibling}W=Uc(ee,Q.mode,Ae),W.return=Q,Q=W}return E(Q);case B:return Ze=ee._init,Wt(Q,W,Ze(ee._payload),Ae)}if(U(ee))return ze(Q,W,ee,Ae);if(ue(ee))return Ve(Q,W,ee,Ae);Lo(Q,ee)}return typeof ee=="string"&&ee!==""||typeof ee=="number"?(ee=""+ee,W!==null&&W.tag===6?(a(Q,W.sibling),W=f(W,ee),W.return=Q,Q=W):(a(Q,W),W=Ic(ee,Q.mode,Ae),W.return=Q,Q=W),E(Q)):a(Q,W)}return Wt}var _s=Dh(!0),Ih=Dh(!1),No=er(null),Do=null,vs=null,Wu=null;function Xu(){Wu=vs=Do=null}function ju(n){var i=No.current;Ft(No),n._currentValue=i}function Yu(n,i,a){for(;n!==null;){var l=n.alternate;if((n.childLanes&i)!==i?(n.childLanes|=i,l!==null&&(l.childLanes|=i)):l!==null&&(l.childLanes&i)!==i&&(l.childLanes|=i),n===a)break;n=n.return}}function xs(n,i){Do=n,Wu=vs=null,n=n.dependencies,n!==null&&n.firstContext!==null&&((n.lanes&i)!==0&&(Cn=!0),n.firstContext=null)}function Wn(n){var i=n._currentValue;if(Wu!==n)if(n={context:n,memoizedValue:i,next:null},vs===null){if(Do===null)throw Error(t(308));vs=n,Do.dependencies={lanes:0,firstContext:n}}else vs=vs.next=n;return i}var Lr=null;function qu(n){Lr===null?Lr=[n]:Lr.push(n)}function Uh(n,i,a,l){var f=i.interleaved;return f===null?(a.next=a,qu(i)):(a.next=f.next,f.next=a),i.interleaved=a,Ri(n,l)}function Ri(n,i){n.lanes|=i;var a=n.alternate;for(a!==null&&(a.lanes|=i),a=n,n=n.return;n!==null;)n.childLanes|=i,a=n.alternate,a!==null&&(a.childLanes|=i),a=n,n=n.return;return a.tag===3?a.stateNode:null}var ir=!1;function $u(n){n.updateQueue={baseState:n.memoizedState,firstBaseUpdate:null,lastBaseUpdate:null,shared:{pending:null,interleaved:null,lanes:0},effects:null}}function Fh(n,i){n=n.updateQueue,i.updateQueue===n&&(i.updateQueue={baseState:n.baseState,firstBaseUpdate:n.firstBaseUpdate,lastBaseUpdate:n.lastBaseUpdate,shared:n.shared,effects:n.effects})}function bi(n,i){return{eventTime:n,lane:i,tag:0,payload:null,callback:null,next:null}}function rr(n,i,a){var l=n.updateQueue;if(l===null)return null;if(l=l.shared,(St&2)!==0){var f=l.pending;return f===null?i.next=i:(i.next=f.next,f.next=i),l.pending=i,Ri(n,a)}return f=l.interleaved,f===null?(i.next=i,qu(l)):(i.next=f.next,f.next=i),l.interleaved=i,Ri(n,a)}function Io(n,i,a){if(i=i.updateQueue,i!==null&&(i=i.shared,(a&4194240)!==0)){var l=i.lanes;l&=n.pendingLanes,a|=l,i.lanes=a,uu(n,a)}}function Oh(n,i){var a=n.updateQueue,l=n.alternate;if(l!==null&&(l=l.updateQueue,a===l)){var f=null,p=null;if(a=a.firstBaseUpdate,a!==null){do{var E={eventTime:a.eventTime,lane:a.lane,tag:a.tag,payload:a.payload,callback:a.callback,next:null};p===null?f=p=E:p=p.next=E,a=a.next}while(a!==null);p===null?f=p=i:p=p.next=i}else f=p=i;a={baseState:l.baseState,firstBaseUpdate:f,lastBaseUpdate:p,shared:l.shared,effects:l.effects},n.updateQueue=a;return}n=a.lastBaseUpdate,n===null?a.firstBaseUpdate=i:n.next=i,a.lastBaseUpdate=i}function Uo(n,i,a,l){var f=n.updateQueue;ir=!1;var p=f.firstBaseUpdate,E=f.lastBaseUpdate,N=f.shared.pending;if(N!==null){f.shared.pending=null;var H=N,re=H.next;H.next=null,E===null?p=re:E.next=re,E=H;var ve=n.alternate;ve!==null&&(ve=ve.updateQueue,N=ve.lastBaseUpdate,N!==E&&(N===null?ve.firstBaseUpdate=re:N.next=re,ve.lastBaseUpdate=H))}if(p!==null){var Se=f.baseState;E=0,ve=re=H=null,N=p;do{var ge=N.lane,Fe=N.eventTime;if((l&ge)===ge){ve!==null&&(ve=ve.next={eventTime:Fe,lane:0,tag:N.tag,payload:N.payload,callback:N.callback,next:null});e:{var ze=n,Ve=N;switch(ge=i,Fe=a,Ve.tag){case 1:if(ze=Ve.payload,typeof ze=="function"){Se=ze.call(Fe,Se,ge);break e}Se=ze;break e;case 3:ze.flags=ze.flags&-65537|128;case 0:if(ze=Ve.payload,ge=typeof ze=="function"?ze.call(Fe,Se,ge):ze,ge==null)break e;Se=le({},Se,ge);break e;case 2:ir=!0}}N.callback!==null&&N.lane!==0&&(n.flags|=64,ge=f.effects,ge===null?f.effects=[N]:ge.push(N))}else Fe={eventTime:Fe,lane:ge,tag:N.tag,payload:N.payload,callback:N.callback,next:null},ve===null?(re=ve=Fe,H=Se):ve=ve.next=Fe,E|=ge;if(N=N.next,N===null){if(N=f.shared.pending,N===null)break;ge=N,N=ge.next,ge.next=null,f.lastBaseUpdate=ge,f.shared.pending=null}}while(!0);if(ve===null&&(H=Se),f.baseState=H,f.firstBaseUpdate=re,f.lastBaseUpdate=ve,i=f.shared.interleaved,i!==null){f=i;do E|=f.lane,f=f.next;while(f!==i)}else p===null&&(f.shared.lanes=0);Ir|=E,n.lanes=E,n.memoizedState=Se}}function kh(n,i,a){if(n=i.effects,i.effects=null,n!==null)for(i=0;i<n.length;i++){var l=n[i],f=l.callback;if(f!==null){if(l.callback=null,l=a,typeof f!="function")throw Error(t(191,f));f.call(l)}}}var Ra={},pi=er(Ra),ba=er(Ra),Pa=er(Ra);function Nr(n){if(n===Ra)throw Error(t(174));return n}function Ku(n,i){switch(Dt(Pa,i),Dt(ba,n),Dt(pi,Ra),n=i.nodeType,n){case 9:case 11:i=(i=i.documentElement)?i.namespaceURI:be(null,"");break;default:n=n===8?i.parentNode:i,i=n.namespaceURI||null,n=n.tagName,i=be(i,n)}Ft(pi),Dt(pi,i)}function ys(){Ft(pi),Ft(ba),Ft(Pa)}function Bh(n){Nr(Pa.current);var i=Nr(pi.current),a=be(i,n.type);i!==a&&(Dt(ba,n),Dt(pi,a))}function Zu(n){ba.current===n&&(Ft(pi),Ft(ba))}var zt=er(0);function Fo(n){for(var i=n;i!==null;){if(i.tag===13){var a=i.memoizedState;if(a!==null&&(a=a.dehydrated,a===null||a.data==="$?"||a.data==="$!"))return i}else if(i.tag===19&&i.memoizedProps.revealOrder!==void 0){if((i.flags&128)!==0)return i}else if(i.child!==null){i.child.return=i,i=i.child;continue}if(i===n)break;for(;i.sibling===null;){if(i.return===null||i.return===n)return null;i=i.return}i.sibling.return=i.return,i=i.sibling}return null}var Qu=[];function Ju(){for(var n=0;n<Qu.length;n++)Qu[n]._workInProgressVersionPrimary=null;Qu.length=0}var Oo=L.ReactCurrentDispatcher,ec=L.ReactCurrentBatchConfig,Dr=0,Ht=null,$t=null,rn=null,ko=!1,La=!1,Na=0,fv=0;function fn(){throw Error(t(321))}function tc(n,i){if(i===null)return!1;for(var a=0;a<i.length&&a<n.length;a++)if(!Jn(n[a],i[a]))return!1;return!0}function nc(n,i,a,l,f,p){if(Dr=p,Ht=i,i.memoizedState=null,i.updateQueue=null,i.lanes=0,Oo.current=n===null||n.memoizedState===null?mv:gv,n=a(l,f),La){p=0;do{if(La=!1,Na=0,25<=p)throw Error(t(301));p+=1,rn=$t=null,i.updateQueue=null,Oo.current=_v,n=a(l,f)}while(La)}if(Oo.current=Ho,i=$t!==null&&$t.next!==null,Dr=0,rn=$t=Ht=null,ko=!1,i)throw Error(t(300));return n}function ic(){var n=Na!==0;return Na=0,n}function mi(){var n={memoizedState:null,baseState:null,baseQueue:null,queue:null,next:null};return rn===null?Ht.memoizedState=rn=n:rn=rn.next=n,rn}function Xn(){if($t===null){var n=Ht.alternate;n=n!==null?n.memoizedState:null}else n=$t.next;var i=rn===null?Ht.memoizedState:rn.next;if(i!==null)rn=i,$t=n;else{if(n===null)throw Error(t(310));$t=n,n={memoizedState:$t.memoizedState,baseState:$t.baseState,baseQueue:$t.baseQueue,queue:$t.queue,next:null},rn===null?Ht.memoizedState=rn=n:rn=rn.next=n}return rn}function Da(n,i){return typeof i=="function"?i(n):i}function rc(n){var i=Xn(),a=i.queue;if(a===null)throw Error(t(311));a.lastRenderedReducer=n;var l=$t,f=l.baseQueue,p=a.pending;if(p!==null){if(f!==null){var E=f.next;f.next=p.next,p.next=E}l.baseQueue=f=p,a.pending=null}if(f!==null){p=f.next,l=l.baseState;var N=E=null,H=null,re=p;do{var ve=re.lane;if((Dr&ve)===ve)H!==null&&(H=H.next={lane:0,action:re.action,hasEagerState:re.hasEagerState,eagerState:re.eagerState,next:null}),l=re.hasEagerState?re.eagerState:n(l,re.action);else{var Se={lane:ve,action:re.action,hasEagerState:re.hasEagerState,eagerState:re.eagerState,next:null};H===null?(N=H=Se,E=l):H=H.next=Se,Ht.lanes|=ve,Ir|=ve}re=re.next}while(re!==null&&re!==p);H===null?E=l:H.next=N,Jn(l,i.memoizedState)||(Cn=!0),i.memoizedState=l,i.baseState=E,i.baseQueue=H,a.lastRenderedState=l}if(n=a.interleaved,n!==null){f=n;do p=f.lane,Ht.lanes|=p,Ir|=p,f=f.next;while(f!==n)}else f===null&&(a.lanes=0);return[i.memoizedState,a.dispatch]}function sc(n){var i=Xn(),a=i.queue;if(a===null)throw Error(t(311));a.lastRenderedReducer=n;var l=a.dispatch,f=a.pending,p=i.memoizedState;if(f!==null){a.pending=null;var E=f=f.next;do p=n(p,E.action),E=E.next;while(E!==f);Jn(p,i.memoizedState)||(Cn=!0),i.memoizedState=p,i.baseQueue===null&&(i.baseState=p),a.lastRenderedState=p}return[p,l]}function zh(){}function Hh(n,i){var a=Ht,l=Xn(),f=i(),p=!Jn(l.memoizedState,f);if(p&&(l.memoizedState=f,Cn=!0),l=l.queue,ac(Wh.bind(null,a,l,n),[n]),l.getSnapshot!==i||p||rn!==null&&rn.memoizedState.tag&1){if(a.flags|=2048,Ia(9,Gh.bind(null,a,l,f,i),void 0,null),sn===null)throw Error(t(349));(Dr&30)!==0||Vh(a,i,f)}return f}function Vh(n,i,a){n.flags|=16384,n={getSnapshot:i,value:a},i=Ht.updateQueue,i===null?(i={lastEffect:null,stores:null},Ht.updateQueue=i,i.stores=[n]):(a=i.stores,a===null?i.stores=[n]:a.push(n))}function Gh(n,i,a,l){i.value=a,i.getSnapshot=l,Xh(i)&&jh(n)}function Wh(n,i,a){return a(function(){Xh(i)&&jh(n)})}function Xh(n){var i=n.getSnapshot;n=n.value;try{var a=i();return!Jn(n,a)}catch{return!0}}function jh(n){var i=Ri(n,1);i!==null&&ri(i,n,1,-1)}function Yh(n){var i=mi();return typeof n=="function"&&(n=n()),i.memoizedState=i.baseState=n,n={pending:null,interleaved:null,lanes:0,dispatch:null,lastRenderedReducer:Da,lastRenderedState:n},i.queue=n,n=n.dispatch=pv.bind(null,Ht,n),[i.memoizedState,n]}function Ia(n,i,a,l){return n={tag:n,create:i,destroy:a,deps:l,next:null},i=Ht.updateQueue,i===null?(i={lastEffect:null,stores:null},Ht.updateQueue=i,i.lastEffect=n.next=n):(a=i.lastEffect,a===null?i.lastEffect=n.next=n:(l=a.next,a.next=n,n.next=l,i.lastEffect=n)),n}function qh(){return Xn().memoizedState}function Bo(n,i,a,l){var f=mi();Ht.flags|=n,f.memoizedState=Ia(1|i,a,void 0,l===void 0?null:l)}function zo(n,i,a,l){var f=Xn();l=l===void 0?null:l;var p=void 0;if($t!==null){var E=$t.memoizedState;if(p=E.destroy,l!==null&&tc(l,E.deps)){f.memoizedState=Ia(i,a,p,l);return}}Ht.flags|=n,f.memoizedState=Ia(1|i,a,p,l)}function $h(n,i){return Bo(8390656,8,n,i)}function ac(n,i){return zo(2048,8,n,i)}function Kh(n,i){return zo(4,2,n,i)}function Zh(n,i){return zo(4,4,n,i)}function Qh(n,i){if(typeof i=="function")return n=n(),i(n),function(){i(null)};if(i!=null)return n=n(),i.current=n,function(){i.current=null}}function Jh(n,i,a){return a=a!=null?a.concat([n]):null,zo(4,4,Qh.bind(null,i,n),a)}function oc(){}function ep(n,i){var a=Xn();i=i===void 0?null:i;var l=a.memoizedState;return l!==null&&i!==null&&tc(i,l[1])?l[0]:(a.memoizedState=[n,i],n)}function tp(n,i){var a=Xn();i=i===void 0?null:i;var l=a.memoizedState;return l!==null&&i!==null&&tc(i,l[1])?l[0]:(n=n(),a.memoizedState=[n,i],n)}function np(n,i,a){return(Dr&21)===0?(n.baseState&&(n.baseState=!1,Cn=!0),n.memoizedState=a):(Jn(a,i)||(a=oo(),Ht.lanes|=a,Ir|=a,n.baseState=!0),i)}function dv(n,i){var a=Rt;Rt=a!==0&&4>a?a:4,n(!0);var l=ec.transition;ec.transition={};try{n(!1),i()}finally{Rt=a,ec.transition=l}}function ip(){return Xn().memoizedState}function hv(n,i,a){var l=lr(n);if(a={lane:l,action:a,hasEagerState:!1,eagerState:null,next:null},rp(n))sp(i,a);else if(a=Uh(n,i,a,l),a!==null){var f=xn();ri(a,n,l,f),ap(a,i,l)}}function pv(n,i,a){var l=lr(n),f={lane:l,action:a,hasEagerState:!1,eagerState:null,next:null};if(rp(n))sp(i,f);else{var p=n.alternate;if(n.lanes===0&&(p===null||p.lanes===0)&&(p=i.lastRenderedReducer,p!==null))try{var E=i.lastRenderedState,N=p(E,a);if(f.hasEagerState=!0,f.eagerState=N,Jn(N,E)){var H=i.interleaved;H===null?(f.next=f,qu(i)):(f.next=H.next,H.next=f),i.interleaved=f;return}}catch{}finally{}a=Uh(n,i,f,l),a!==null&&(f=xn(),ri(a,n,l,f),ap(a,i,l))}}function rp(n){var i=n.alternate;return n===Ht||i!==null&&i===Ht}function sp(n,i){La=ko=!0;var a=n.pending;a===null?i.next=i:(i.next=a.next,a.next=i),n.pending=i}function ap(n,i,a){if((a&4194240)!==0){var l=i.lanes;l&=n.pendingLanes,a|=l,i.lanes=a,uu(n,a)}}var Ho={readContext:Wn,useCallback:fn,useContext:fn,useEffect:fn,useImperativeHandle:fn,useInsertionEffect:fn,useLayoutEffect:fn,useMemo:fn,useReducer:fn,useRef:fn,useState:fn,useDebugValue:fn,useDeferredValue:fn,useTransition:fn,useMutableSource:fn,useSyncExternalStore:fn,useId:fn,unstable_isNewReconciler:!1},mv={readContext:Wn,useCallback:function(n,i){return mi().memoizedState=[n,i===void 0?null:i],n},useContext:Wn,useEffect:$h,useImperativeHandle:function(n,i,a){return a=a!=null?a.concat([n]):null,Bo(4194308,4,Qh.bind(null,i,n),a)},useLayoutEffect:function(n,i){return Bo(4194308,4,n,i)},useInsertionEffect:function(n,i){return Bo(4,2,n,i)},useMemo:function(n,i){var a=mi();return i=i===void 0?null:i,n=n(),a.memoizedState=[n,i],n},useReducer:function(n,i,a){var l=mi();return i=a!==void 0?a(i):i,l.memoizedState=l.baseState=i,n={pending:null,interleaved:null,lanes:0,dispatch:null,lastRenderedReducer:n,lastRenderedState:i},l.queue=n,n=n.dispatch=hv.bind(null,Ht,n),[l.memoizedState,n]},useRef:function(n){var i=mi();return n={current:n},i.memoizedState=n},useState:Yh,useDebugValue:oc,useDeferredValue:function(n){return mi().memoizedState=n},useTransition:function(){var n=Yh(!1),i=n[0];return n=dv.bind(null,n[1]),mi().memoizedState=n,[i,n]},useMutableSource:function(){},useSyncExternalStore:function(n,i,a){var l=Ht,f=mi();if(Bt){if(a===void 0)throw Error(t(407));a=a()}else{if(a=i(),sn===null)throw Error(t(349));(Dr&30)!==0||Vh(l,i,a)}f.memoizedState=a;var p={value:a,getSnapshot:i};return f.queue=p,$h(Wh.bind(null,l,p,n),[n]),l.flags|=2048,Ia(9,Gh.bind(null,l,p,a,i),void 0,null),a},useId:function(){var n=mi(),i=sn.identifierPrefix;if(Bt){var a=Ci,l=Ai;a=(l&~(1<<32-Ct(l)-1)).toString(32)+a,i=":"+i+"R"+a,a=Na++,0<a&&(i+="H"+a.toString(32)),i+=":"}else a=fv++,i=":"+i+"r"+a.toString(32)+":";return n.memoizedState=i},unstable_isNewReconciler:!1},gv={readContext:Wn,useCallback:ep,useContext:Wn,useEffect:ac,useImperativeHandle:Jh,useInsertionEffect:Kh,useLayoutEffect:Zh,useMemo:tp,useReducer:rc,useRef:qh,useState:function(){return rc(Da)},useDebugValue:oc,useDeferredValue:function(n){var i=Xn();return np(i,$t.memoizedState,n)},useTransition:function(){var n=rc(Da)[0],i=Xn().memoizedState;return[n,i]},useMutableSource:zh,useSyncExternalStore:Hh,useId:ip,unstable_isNewReconciler:!1},_v={readContext:Wn,useCallback:ep,useContext:Wn,useEffect:ac,useImperativeHandle:Jh,useInsertionEffect:Kh,useLayoutEffect:Zh,useMemo:tp,useReducer:sc,useRef:qh,useState:function(){return sc(Da)},useDebugValue:oc,useDeferredValue:function(n){var i=Xn();return $t===null?i.memoizedState=n:np(i,$t.memoizedState,n)},useTransition:function(){var n=sc(Da)[0],i=Xn().memoizedState;return[n,i]},useMutableSource:zh,useSyncExternalStore:Hh,useId:ip,unstable_isNewReconciler:!1};function ti(n,i){if(n&&n.defaultProps){i=le({},i),n=n.defaultProps;for(var a in n)i[a]===void 0&&(i[a]=n[a]);return i}return i}function lc(n,i,a,l){i=n.memoizedState,a=a(l,i),a=a==null?i:le({},i,a),n.memoizedState=a,n.lanes===0&&(n.updateQueue.baseState=a)}var Vo={isMounted:function(n){return(n=n._reactInternals)?Ei(n)===n:!1},enqueueSetState:function(n,i,a){n=n._reactInternals;var l=xn(),f=lr(n),p=bi(l,f);p.payload=i,a!=null&&(p.callback=a),i=rr(n,p,f),i!==null&&(ri(i,n,f,l),Io(i,n,f))},enqueueReplaceState:function(n,i,a){n=n._reactInternals;var l=xn(),f=lr(n),p=bi(l,f);p.tag=1,p.payload=i,a!=null&&(p.callback=a),i=rr(n,p,f),i!==null&&(ri(i,n,f,l),Io(i,n,f))},enqueueForceUpdate:function(n,i){n=n._reactInternals;var a=xn(),l=lr(n),f=bi(a,l);f.tag=2,i!=null&&(f.callback=i),i=rr(n,f,l),i!==null&&(ri(i,n,l,a),Io(i,n,l))}};function op(n,i,a,l,f,p,E){return n=n.stateNode,typeof n.shouldComponentUpdate=="function"?n.shouldComponentUpdate(l,p,E):i.prototype&&i.prototype.isPureReactComponent?!ya(a,l)||!ya(f,p):!0}function lp(n,i,a){var l=!1,f=tr,p=i.contextType;return typeof p=="object"&&p!==null?p=Wn(p):(f=An(i)?Rr:cn.current,l=i.contextTypes,p=(l=l!=null)?hs(n,f):tr),i=new i(a,p),n.memoizedState=i.state!==null&&i.state!==void 0?i.state:null,i.updater=Vo,n.stateNode=i,i._reactInternals=n,l&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=f,n.__reactInternalMemoizedMaskedChildContext=p),i}function up(n,i,a,l){n=i.state,typeof i.componentWillReceiveProps=="function"&&i.componentWillReceiveProps(a,l),typeof i.UNSAFE_componentWillReceiveProps=="function"&&i.UNSAFE_componentWillReceiveProps(a,l),i.state!==n&&Vo.enqueueReplaceState(i,i.state,null)}function uc(n,i,a,l){var f=n.stateNode;f.props=a,f.state=n.memoizedState,f.refs={},$u(n);var p=i.contextType;typeof p=="object"&&p!==null?f.context=Wn(p):(p=An(i)?Rr:cn.current,f.context=hs(n,p)),f.state=n.memoizedState,p=i.getDerivedStateFromProps,typeof p=="function"&&(lc(n,i,p,a),f.state=n.memoizedState),typeof i.getDerivedStateFromProps=="function"||typeof f.getSnapshotBeforeUpdate=="function"||typeof f.UNSAFE_componentWillMount!="function"&&typeof f.componentWillMount!="function"||(i=f.state,typeof f.componentWillMount=="function"&&f.componentWillMount(),typeof f.UNSAFE_componentWillMount=="function"&&f.UNSAFE_componentWillMount(),i!==f.state&&Vo.enqueueReplaceState(f,f.state,null),Uo(n,a,f,l),f.state=n.memoizedState),typeof f.componentDidMount=="function"&&(n.flags|=4194308)}function Ss(n,i){try{var a="",l=i;do a+=fe(l),l=l.return;while(l);var f=a}catch(p){f=`
Error generating stack: `+p.message+`
`+p.stack}return{value:n,source:i,stack:f,digest:null}}function cc(n,i,a){return{value:n,source:null,stack:a??null,digest:i??null}}function fc(n,i){try{console.error(i.value)}catch(a){setTimeout(function(){throw a})}}var vv=typeof WeakMap=="function"?WeakMap:Map;function cp(n,i,a){a=bi(-1,a),a.tag=3,a.payload={element:null};var l=i.value;return a.callback=function(){$o||($o=!0,Ac=l),fc(n,i)},a}function fp(n,i,a){a=bi(-1,a),a.tag=3;var l=n.type.getDerivedStateFromError;if(typeof l=="function"){var f=i.value;a.payload=function(){return l(f)},a.callback=function(){fc(n,i)}}var p=n.stateNode;return p!==null&&typeof p.componentDidCatch=="function"&&(a.callback=function(){fc(n,i),typeof l!="function"&&(ar===null?ar=new Set([this]):ar.add(this));var E=i.stack;this.componentDidCatch(i.value,{componentStack:E!==null?E:""})}),a}function dp(n,i,a){var l=n.pingCache;if(l===null){l=n.pingCache=new vv;var f=new Set;l.set(i,f)}else f=l.get(i),f===void 0&&(f=new Set,l.set(i,f));f.has(a)||(f.add(a),n=Nv.bind(null,n,i,a),i.then(n,n))}function hp(n){do{var i;if((i=n.tag===13)&&(i=n.memoizedState,i=i!==null?i.dehydrated!==null:!0),i)return n;n=n.return}while(n!==null);return null}function pp(n,i,a,l,f){return(n.mode&1)===0?(n===i?n.flags|=65536:(n.flags|=128,a.flags|=131072,a.flags&=-52805,a.tag===1&&(a.alternate===null?a.tag=17:(i=bi(-1,1),i.tag=2,rr(a,i,1))),a.lanes|=1),n):(n.flags|=65536,n.lanes=f,n)}var xv=L.ReactCurrentOwner,Cn=!1;function vn(n,i,a,l){i.child=n===null?Ih(i,null,a,l):_s(i,n.child,a,l)}function mp(n,i,a,l,f){a=a.render;var p=i.ref;return xs(i,f),l=nc(n,i,a,l,p,f),a=ic(),n!==null&&!Cn?(i.updateQueue=n.updateQueue,i.flags&=-2053,n.lanes&=~f,Pi(n,i,f)):(Bt&&a&&Bu(i),i.flags|=1,vn(n,i,l,f),i.child)}function gp(n,i,a,l,f){if(n===null){var p=a.type;return typeof p=="function"&&!Dc(p)&&p.defaultProps===void 0&&a.compare===null&&a.defaultProps===void 0?(i.tag=15,i.type=p,_p(n,i,p,l,f)):(n=tl(a.type,null,l,i,i.mode,f),n.ref=i.ref,n.return=i,i.child=n)}if(p=n.child,(n.lanes&f)===0){var E=p.memoizedProps;if(a=a.compare,a=a!==null?a:ya,a(E,l)&&n.ref===i.ref)return Pi(n,i,f)}return i.flags|=1,n=cr(p,l),n.ref=i.ref,n.return=i,i.child=n}function _p(n,i,a,l,f){if(n!==null){var p=n.memoizedProps;if(ya(p,l)&&n.ref===i.ref)if(Cn=!1,i.pendingProps=l=p,(n.lanes&f)!==0)(n.flags&131072)!==0&&(Cn=!0);else return i.lanes=n.lanes,Pi(n,i,f)}return dc(n,i,a,l,f)}function vp(n,i,a){var l=i.pendingProps,f=l.children,p=n!==null?n.memoizedState:null;if(l.mode==="hidden")if((i.mode&1)===0)i.memoizedState={baseLanes:0,cachePool:null,transitions:null},Dt(Es,On),On|=a;else{if((a&1073741824)===0)return n=p!==null?p.baseLanes|a:a,i.lanes=i.childLanes=1073741824,i.memoizedState={baseLanes:n,cachePool:null,transitions:null},i.updateQueue=null,Dt(Es,On),On|=n,null;i.memoizedState={baseLanes:0,cachePool:null,transitions:null},l=p!==null?p.baseLanes:a,Dt(Es,On),On|=l}else p!==null?(l=p.baseLanes|a,i.memoizedState=null):l=a,Dt(Es,On),On|=l;return vn(n,i,f,a),i.child}function xp(n,i){var a=i.ref;(n===null&&a!==null||n!==null&&n.ref!==a)&&(i.flags|=512,i.flags|=2097152)}function dc(n,i,a,l,f){var p=An(a)?Rr:cn.current;return p=hs(i,p),xs(i,f),a=nc(n,i,a,l,p,f),l=ic(),n!==null&&!Cn?(i.updateQueue=n.updateQueue,i.flags&=-2053,n.lanes&=~f,Pi(n,i,f)):(Bt&&l&&Bu(i),i.flags|=1,vn(n,i,a,f),i.child)}function yp(n,i,a,l,f){if(An(a)){var p=!0;Ao(i)}else p=!1;if(xs(i,f),i.stateNode===null)Wo(n,i),lp(i,a,l),uc(i,a,l,f),l=!0;else if(n===null){var E=i.stateNode,N=i.memoizedProps;E.props=N;var H=E.context,re=a.contextType;typeof re=="object"&&re!==null?re=Wn(re):(re=An(a)?Rr:cn.current,re=hs(i,re));var ve=a.getDerivedStateFromProps,Se=typeof ve=="function"||typeof E.getSnapshotBeforeUpdate=="function";Se||typeof E.UNSAFE_componentWillReceiveProps!="function"&&typeof E.componentWillReceiveProps!="function"||(N!==l||H!==re)&&up(i,E,l,re),ir=!1;var ge=i.memoizedState;E.state=ge,Uo(i,l,E,f),H=i.memoizedState,N!==l||ge!==H||Tn.current||ir?(typeof ve=="function"&&(lc(i,a,ve,l),H=i.memoizedState),(N=ir||op(i,a,N,l,ge,H,re))?(Se||typeof E.UNSAFE_componentWillMount!="function"&&typeof E.componentWillMount!="function"||(typeof E.componentWillMount=="function"&&E.componentWillMount(),typeof E.UNSAFE_componentWillMount=="function"&&E.UNSAFE_componentWillMount()),typeof E.componentDidMount=="function"&&(i.flags|=4194308)):(typeof E.componentDidMount=="function"&&(i.flags|=4194308),i.memoizedProps=l,i.memoizedState=H),E.props=l,E.state=H,E.context=re,l=N):(typeof E.componentDidMount=="function"&&(i.flags|=4194308),l=!1)}else{E=i.stateNode,Fh(n,i),N=i.memoizedProps,re=i.type===i.elementType?N:ti(i.type,N),E.props=re,Se=i.pendingProps,ge=E.context,H=a.contextType,typeof H=="object"&&H!==null?H=Wn(H):(H=An(a)?Rr:cn.current,H=hs(i,H));var Fe=a.getDerivedStateFromProps;(ve=typeof Fe=="function"||typeof E.getSnapshotBeforeUpdate=="function")||typeof E.UNSAFE_componentWillReceiveProps!="function"&&typeof E.componentWillReceiveProps!="function"||(N!==Se||ge!==H)&&up(i,E,l,H),ir=!1,ge=i.memoizedState,E.state=ge,Uo(i,l,E,f);var ze=i.memoizedState;N!==Se||ge!==ze||Tn.current||ir?(typeof Fe=="function"&&(lc(i,a,Fe,l),ze=i.memoizedState),(re=ir||op(i,a,re,l,ge,ze,H)||!1)?(ve||typeof E.UNSAFE_componentWillUpdate!="function"&&typeof E.componentWillUpdate!="function"||(typeof E.componentWillUpdate=="function"&&E.componentWillUpdate(l,ze,H),typeof E.UNSAFE_componentWillUpdate=="function"&&E.UNSAFE_componentWillUpdate(l,ze,H)),typeof E.componentDidUpdate=="function"&&(i.flags|=4),typeof E.getSnapshotBeforeUpdate=="function"&&(i.flags|=1024)):(typeof E.componentDidUpdate!="function"||N===n.memoizedProps&&ge===n.memoizedState||(i.flags|=4),typeof E.getSnapshotBeforeUpdate!="function"||N===n.memoizedProps&&ge===n.memoizedState||(i.flags|=1024),i.memoizedProps=l,i.memoizedState=ze),E.props=l,E.state=ze,E.context=H,l=re):(typeof E.componentDidUpdate!="function"||N===n.memoizedProps&&ge===n.memoizedState||(i.flags|=4),typeof E.getSnapshotBeforeUpdate!="function"||N===n.memoizedProps&&ge===n.memoizedState||(i.flags|=1024),l=!1)}return hc(n,i,a,l,p,f)}function hc(n,i,a,l,f,p){xp(n,i);var E=(i.flags&128)!==0;if(!l&&!E)return f&&Th(i,a,!1),Pi(n,i,p);l=i.stateNode,xv.current=i;var N=E&&typeof a.getDerivedStateFromError!="function"?null:l.render();return i.flags|=1,n!==null&&E?(i.child=_s(i,n.child,null,p),i.child=_s(i,null,N,p)):vn(n,i,N,p),i.memoizedState=l.state,f&&Th(i,a,!0),i.child}function Sp(n){var i=n.stateNode;i.pendingContext?Eh(n,i.pendingContext,i.pendingContext!==i.context):i.context&&Eh(n,i.context,!1),Ku(n,i.containerInfo)}function Mp(n,i,a,l,f){return gs(),Gu(f),i.flags|=256,vn(n,i,a,l),i.child}var pc={dehydrated:null,treeContext:null,retryLane:0};function mc(n){return{baseLanes:n,cachePool:null,transitions:null}}function Ep(n,i,a){var l=i.pendingProps,f=zt.current,p=!1,E=(i.flags&128)!==0,N;if((N=E)||(N=n!==null&&n.memoizedState===null?!1:(f&2)!==0),N?(p=!0,i.flags&=-129):(n===null||n.memoizedState!==null)&&(f|=1),Dt(zt,f&1),n===null)return Vu(i),n=i.memoizedState,n!==null&&(n=n.dehydrated,n!==null)?((i.mode&1)===0?i.lanes=1:n.data==="$!"?i.lanes=8:i.lanes=1073741824,null):(E=l.children,n=l.fallback,p?(l=i.mode,p=i.child,E={mode:"hidden",children:E},(l&1)===0&&p!==null?(p.childLanes=0,p.pendingProps=E):p=nl(E,l,0,null),n=kr(n,l,a,null),p.return=i,n.return=i,p.sibling=n,i.child=p,i.child.memoizedState=mc(a),i.memoizedState=pc,n):gc(i,E));if(f=n.memoizedState,f!==null&&(N=f.dehydrated,N!==null))return yv(n,i,E,l,N,f,a);if(p){p=l.fallback,E=i.mode,f=n.child,N=f.sibling;var H={mode:"hidden",children:l.children};return(E&1)===0&&i.child!==f?(l=i.child,l.childLanes=0,l.pendingProps=H,i.deletions=null):(l=cr(f,H),l.subtreeFlags=f.subtreeFlags&14680064),N!==null?p=cr(N,p):(p=kr(p,E,a,null),p.flags|=2),p.return=i,l.return=i,l.sibling=p,i.child=l,l=p,p=i.child,E=n.child.memoizedState,E=E===null?mc(a):{baseLanes:E.baseLanes|a,cachePool:null,transitions:E.transitions},p.memoizedState=E,p.childLanes=n.childLanes&~a,i.memoizedState=pc,l}return p=n.child,n=p.sibling,l=cr(p,{mode:"visible",children:l.children}),(i.mode&1)===0&&(l.lanes=a),l.return=i,l.sibling=null,n!==null&&(a=i.deletions,a===null?(i.deletions=[n],i.flags|=16):a.push(n)),i.child=l,i.memoizedState=null,l}function gc(n,i){return i=nl({mode:"visible",children:i},n.mode,0,null),i.return=n,n.child=i}function Go(n,i,a,l){return l!==null&&Gu(l),_s(i,n.child,null,a),n=gc(i,i.pendingProps.children),n.flags|=2,i.memoizedState=null,n}function yv(n,i,a,l,f,p,E){if(a)return i.flags&256?(i.flags&=-257,l=cc(Error(t(422))),Go(n,i,E,l)):i.memoizedState!==null?(i.child=n.child,i.flags|=128,null):(p=l.fallback,f=i.mode,l=nl({mode:"visible",children:l.children},f,0,null),p=kr(p,f,E,null),p.flags|=2,l.return=i,p.return=i,l.sibling=p,i.child=l,(i.mode&1)!==0&&_s(i,n.child,null,E),i.child.memoizedState=mc(E),i.memoizedState=pc,p);if((i.mode&1)===0)return Go(n,i,E,null);if(f.data==="$!"){if(l=f.nextSibling&&f.nextSibling.dataset,l)var N=l.dgst;return l=N,p=Error(t(419)),l=cc(p,l,void 0),Go(n,i,E,l)}if(N=(E&n.childLanes)!==0,Cn||N){if(l=sn,l!==null){switch(E&-E){case 4:f=2;break;case 16:f=8;break;case 64:case 128:case 256:case 512:case 1024:case 2048:case 4096:case 8192:case 16384:case 32768:case 65536:case 131072:case 262144:case 524288:case 1048576:case 2097152:case 4194304:case 8388608:case 16777216:case 33554432:case 67108864:f=32;break;case 536870912:f=268435456;break;default:f=0}f=(f&(l.suspendedLanes|E))!==0?0:f,f!==0&&f!==p.retryLane&&(p.retryLane=f,Ri(n,f),ri(l,n,f,-1))}return Nc(),l=cc(Error(t(421))),Go(n,i,E,l)}return f.data==="$?"?(i.flags|=128,i.child=n.child,i=Dv.bind(null,n),f._reactRetry=i,null):(n=p.treeContext,Fn=Ji(f.nextSibling),Un=i,Bt=!0,ei=null,n!==null&&(Vn[Gn++]=Ai,Vn[Gn++]=Ci,Vn[Gn++]=br,Ai=n.id,Ci=n.overflow,br=i),i=gc(i,l.children),i.flags|=4096,i)}function wp(n,i,a){n.lanes|=i;var l=n.alternate;l!==null&&(l.lanes|=i),Yu(n.return,i,a)}function _c(n,i,a,l,f){var p=n.memoizedState;p===null?n.memoizedState={isBackwards:i,rendering:null,renderingStartTime:0,last:l,tail:a,tailMode:f}:(p.isBackwards=i,p.rendering=null,p.renderingStartTime=0,p.last=l,p.tail=a,p.tailMode=f)}function Tp(n,i,a){var l=i.pendingProps,f=l.revealOrder,p=l.tail;if(vn(n,i,l.children,a),l=zt.current,(l&2)!==0)l=l&1|2,i.flags|=128;else{if(n!==null&&(n.flags&128)!==0)e:for(n=i.child;n!==null;){if(n.tag===13)n.memoizedState!==null&&wp(n,a,i);else if(n.tag===19)wp(n,a,i);else if(n.child!==null){n.child.return=n,n=n.child;continue}if(n===i)break e;for(;n.sibling===null;){if(n.return===null||n.return===i)break e;n=n.return}n.sibling.return=n.return,n=n.sibling}l&=1}if(Dt(zt,l),(i.mode&1)===0)i.memoizedState=null;else switch(f){case"forwards":for(a=i.child,f=null;a!==null;)n=a.alternate,n!==null&&Fo(n)===null&&(f=a),a=a.sibling;a=f,a===null?(f=i.child,i.child=null):(f=a.sibling,a.sibling=null),_c(i,!1,f,a,p);break;case"backwards":for(a=null,f=i.child,i.child=null;f!==null;){if(n=f.alternate,n!==null&&Fo(n)===null){i.child=f;break}n=f.sibling,f.sibling=a,a=f,f=n}_c(i,!0,a,null,p);break;case"together":_c(i,!1,null,null,void 0);break;default:i.memoizedState=null}return i.child}function Wo(n,i){(i.mode&1)===0&&n!==null&&(n.alternate=null,i.alternate=null,i.flags|=2)}function Pi(n,i,a){if(n!==null&&(i.dependencies=n.dependencies),Ir|=i.lanes,(a&i.childLanes)===0)return null;if(n!==null&&i.child!==n.child)throw Error(t(153));if(i.child!==null){for(n=i.child,a=cr(n,n.pendingProps),i.child=a,a.return=i;n.sibling!==null;)n=n.sibling,a=a.sibling=cr(n,n.pendingProps),a.return=i;a.sibling=null}return i.child}function Sv(n,i,a){switch(i.tag){case 3:Sp(i),gs();break;case 5:Bh(i);break;case 1:An(i.type)&&Ao(i);break;case 4:Ku(i,i.stateNode.containerInfo);break;case 10:var l=i.type._context,f=i.memoizedProps.value;Dt(No,l._currentValue),l._currentValue=f;break;case 13:if(l=i.memoizedState,l!==null)return l.dehydrated!==null?(Dt(zt,zt.current&1),i.flags|=128,null):(a&i.child.childLanes)!==0?Ep(n,i,a):(Dt(zt,zt.current&1),n=Pi(n,i,a),n!==null?n.sibling:null);Dt(zt,zt.current&1);break;case 19:if(l=(a&i.childLanes)!==0,(n.flags&128)!==0){if(l)return Tp(n,i,a);i.flags|=128}if(f=i.memoizedState,f!==null&&(f.rendering=null,f.tail=null,f.lastEffect=null),Dt(zt,zt.current),l)break;return null;case 22:case 23:return i.lanes=0,vp(n,i,a)}return Pi(n,i,a)}var Ap,vc,Cp,Rp;Ap=function(n,i){for(var a=i.child;a!==null;){if(a.tag===5||a.tag===6)n.appendChild(a.stateNode);else if(a.tag!==4&&a.child!==null){a.child.return=a,a=a.child;continue}if(a===i)break;for(;a.sibling===null;){if(a.return===null||a.return===i)return;a=a.return}a.sibling.return=a.return,a=a.sibling}},vc=function(){},Cp=function(n,i,a,l){var f=n.memoizedProps;if(f!==l){n=i.stateNode,Nr(pi.current);var p=null;switch(a){case"input":f=vt(n,f),l=vt(n,l),p=[];break;case"select":f=le({},f,{value:void 0}),l=le({},l,{value:void 0}),p=[];break;case"textarea":f=se(n,f),l=se(n,l),p=[];break;default:typeof f.onClick!="function"&&typeof l.onClick=="function"&&(n.onclick=Eo)}lt(a,l);var E;a=null;for(re in f)if(!l.hasOwnProperty(re)&&f.hasOwnProperty(re)&&f[re]!=null)if(re==="style"){var N=f[re];for(E in N)N.hasOwnProperty(E)&&(a||(a={}),a[E]="")}else re!=="dangerouslySetInnerHTML"&&re!=="children"&&re!=="suppressContentEditableWarning"&&re!=="suppressHydrationWarning"&&re!=="autoFocus"&&(o.hasOwnProperty(re)?p||(p=[]):(p=p||[]).push(re,null));for(re in l){var H=l[re];if(N=f!=null?f[re]:void 0,l.hasOwnProperty(re)&&H!==N&&(H!=null||N!=null))if(re==="style")if(N){for(E in N)!N.hasOwnProperty(E)||H&&H.hasOwnProperty(E)||(a||(a={}),a[E]="");for(E in H)H.hasOwnProperty(E)&&N[E]!==H[E]&&(a||(a={}),a[E]=H[E])}else a||(p||(p=[]),p.push(re,a)),a=H;else re==="dangerouslySetInnerHTML"?(H=H?H.__html:void 0,N=N?N.__html:void 0,H!=null&&N!==H&&(p=p||[]).push(re,H)):re==="children"?typeof H!="string"&&typeof H!="number"||(p=p||[]).push(re,""+H):re!=="suppressContentEditableWarning"&&re!=="suppressHydrationWarning"&&(o.hasOwnProperty(re)?(H!=null&&re==="onScroll"&&Ut("scroll",n),p||N===H||(p=[])):(p=p||[]).push(re,H))}a&&(p=p||[]).push("style",a);var re=p;(i.updateQueue=re)&&(i.flags|=4)}},Rp=function(n,i,a,l){a!==l&&(i.flags|=4)};function Ua(n,i){if(!Bt)switch(n.tailMode){case"hidden":i=n.tail;for(var a=null;i!==null;)i.alternate!==null&&(a=i),i=i.sibling;a===null?n.tail=null:a.sibling=null;break;case"collapsed":a=n.tail;for(var l=null;a!==null;)a.alternate!==null&&(l=a),a=a.sibling;l===null?i||n.tail===null?n.tail=null:n.tail.sibling=null:l.sibling=null}}function dn(n){var i=n.alternate!==null&&n.alternate.child===n.child,a=0,l=0;if(i)for(var f=n.child;f!==null;)a|=f.lanes|f.childLanes,l|=f.subtreeFlags&14680064,l|=f.flags&14680064,f.return=n,f=f.sibling;else for(f=n.child;f!==null;)a|=f.lanes|f.childLanes,l|=f.subtreeFlags,l|=f.flags,f.return=n,f=f.sibling;return n.subtreeFlags|=l,n.childLanes=a,i}function Mv(n,i,a){var l=i.pendingProps;switch(zu(i),i.tag){case 2:case 16:case 15:case 0:case 11:case 7:case 8:case 12:case 9:case 14:return dn(i),null;case 1:return An(i.type)&&To(),dn(i),null;case 3:return l=i.stateNode,ys(),Ft(Tn),Ft(cn),Ju(),l.pendingContext&&(l.context=l.pendingContext,l.pendingContext=null),(n===null||n.child===null)&&(Po(i)?i.flags|=4:n===null||n.memoizedState.isDehydrated&&(i.flags&256)===0||(i.flags|=1024,ei!==null&&(bc(ei),ei=null))),vc(n,i),dn(i),null;case 5:Zu(i);var f=Nr(Pa.current);if(a=i.type,n!==null&&i.stateNode!=null)Cp(n,i,a,l,f),n.ref!==i.ref&&(i.flags|=512,i.flags|=2097152);else{if(!l){if(i.stateNode===null)throw Error(t(166));return dn(i),null}if(n=Nr(pi.current),Po(i)){l=i.stateNode,a=i.type;var p=i.memoizedProps;switch(l[hi]=i,l[Ta]=p,n=(i.mode&1)!==0,a){case"dialog":Ut("cancel",l),Ut("close",l);break;case"iframe":case"object":case"embed":Ut("load",l);break;case"video":case"audio":for(f=0;f<Ma.length;f++)Ut(Ma[f],l);break;case"source":Ut("error",l);break;case"img":case"image":case"link":Ut("error",l),Ut("load",l);break;case"details":Ut("toggle",l);break;case"input":yt(l,p),Ut("invalid",l);break;case"select":l._wrapperState={wasMultiple:!!p.multiple},Ut("invalid",l);break;case"textarea":_e(l,p),Ut("invalid",l)}lt(a,p),f=null;for(var E in p)if(p.hasOwnProperty(E)){var N=p[E];E==="children"?typeof N=="string"?l.textContent!==N&&(p.suppressHydrationWarning!==!0&&Mo(l.textContent,N,n),f=["children",N]):typeof N=="number"&&l.textContent!==""+N&&(p.suppressHydrationWarning!==!0&&Mo(l.textContent,N,n),f=["children",""+N]):o.hasOwnProperty(E)&&N!=null&&E==="onScroll"&&Ut("scroll",l)}switch(a){case"input":wt(l),tt(l,p,!0);break;case"textarea":wt(l),me(l);break;case"select":case"option":break;default:typeof p.onClick=="function"&&(l.onclick=Eo)}l=f,i.updateQueue=l,l!==null&&(i.flags|=4)}else{E=f.nodeType===9?f:f.ownerDocument,n==="http://www.w3.org/1999/xhtml"&&(n=je(a)),n==="http://www.w3.org/1999/xhtml"?a==="script"?(n=E.createElement("div"),n.innerHTML="<script><\/script>",n=n.removeChild(n.firstChild)):typeof l.is=="string"?n=E.createElement(a,{is:l.is}):(n=E.createElement(a),a==="select"&&(E=n,l.multiple?E.multiple=!0:l.size&&(E.size=l.size))):n=E.createElementNS(n,a),n[hi]=i,n[Ta]=l,Ap(n,i,!1,!1),i.stateNode=n;e:{switch(E=Tt(a,l),a){case"dialog":Ut("cancel",n),Ut("close",n),f=l;break;case"iframe":case"object":case"embed":Ut("load",n),f=l;break;case"video":case"audio":for(f=0;f<Ma.length;f++)Ut(Ma[f],n);f=l;break;case"source":Ut("error",n),f=l;break;case"img":case"image":case"link":Ut("error",n),Ut("load",n),f=l;break;case"details":Ut("toggle",n),f=l;break;case"input":yt(n,l),f=vt(n,l),Ut("invalid",n);break;case"option":f=l;break;case"select":n._wrapperState={wasMultiple:!!l.multiple},f=le({},l,{value:void 0}),Ut("invalid",n);break;case"textarea":_e(n,l),f=se(n,l),Ut("invalid",n);break;default:f=l}lt(a,f),N=f;for(p in N)if(N.hasOwnProperty(p)){var H=N[p];p==="style"?Oe(n,H):p==="dangerouslySetInnerHTML"?(H=H?H.__html:void 0,H!=null&&ot(n,H)):p==="children"?typeof H=="string"?(a!=="textarea"||H!=="")&&Ee(n,H):typeof H=="number"&&Ee(n,""+H):p!=="suppressContentEditableWarning"&&p!=="suppressHydrationWarning"&&p!=="autoFocus"&&(o.hasOwnProperty(p)?H!=null&&p==="onScroll"&&Ut("scroll",n):H!=null&&R(n,p,H,E))}switch(a){case"input":wt(n),tt(n,l,!1);break;case"textarea":wt(n),me(n);break;case"option":l.value!=null&&n.setAttribute("value",""+Le(l.value));break;case"select":n.multiple=!!l.multiple,p=l.value,p!=null?A(n,!!l.multiple,p,!1):l.defaultValue!=null&&A(n,!!l.multiple,l.defaultValue,!0);break;default:typeof f.onClick=="function"&&(n.onclick=Eo)}switch(a){case"button":case"input":case"select":case"textarea":l=!!l.autoFocus;break e;case"img":l=!0;break e;default:l=!1}}l&&(i.flags|=4)}i.ref!==null&&(i.flags|=512,i.flags|=2097152)}return dn(i),null;case 6:if(n&&i.stateNode!=null)Rp(n,i,n.memoizedProps,l);else{if(typeof l!="string"&&i.stateNode===null)throw Error(t(166));if(a=Nr(Pa.current),Nr(pi.current),Po(i)){if(l=i.stateNode,a=i.memoizedProps,l[hi]=i,(p=l.nodeValue!==a)&&(n=Un,n!==null))switch(n.tag){case 3:Mo(l.nodeValue,a,(n.mode&1)!==0);break;case 5:n.memoizedProps.suppressHydrationWarning!==!0&&Mo(l.nodeValue,a,(n.mode&1)!==0)}p&&(i.flags|=4)}else l=(a.nodeType===9?a:a.ownerDocument).createTextNode(l),l[hi]=i,i.stateNode=l}return dn(i),null;case 13:if(Ft(zt),l=i.memoizedState,n===null||n.memoizedState!==null&&n.memoizedState.dehydrated!==null){if(Bt&&Fn!==null&&(i.mode&1)!==0&&(i.flags&128)===0)Lh(),gs(),i.flags|=98560,p=!1;else if(p=Po(i),l!==null&&l.dehydrated!==null){if(n===null){if(!p)throw Error(t(318));if(p=i.memoizedState,p=p!==null?p.dehydrated:null,!p)throw Error(t(317));p[hi]=i}else gs(),(i.flags&128)===0&&(i.memoizedState=null),i.flags|=4;dn(i),p=!1}else ei!==null&&(bc(ei),ei=null),p=!0;if(!p)return i.flags&65536?i:null}return(i.flags&128)!==0?(i.lanes=a,i):(l=l!==null,l!==(n!==null&&n.memoizedState!==null)&&l&&(i.child.flags|=8192,(i.mode&1)!==0&&(n===null||(zt.current&1)!==0?Kt===0&&(Kt=3):Nc())),i.updateQueue!==null&&(i.flags|=4),dn(i),null);case 4:return ys(),vc(n,i),n===null&&Ea(i.stateNode.containerInfo),dn(i),null;case 10:return ju(i.type._context),dn(i),null;case 17:return An(i.type)&&To(),dn(i),null;case 19:if(Ft(zt),p=i.memoizedState,p===null)return dn(i),null;if(l=(i.flags&128)!==0,E=p.rendering,E===null)if(l)Ua(p,!1);else{if(Kt!==0||n!==null&&(n.flags&128)!==0)for(n=i.child;n!==null;){if(E=Fo(n),E!==null){for(i.flags|=128,Ua(p,!1),l=E.updateQueue,l!==null&&(i.updateQueue=l,i.flags|=4),i.subtreeFlags=0,l=a,a=i.child;a!==null;)p=a,n=l,p.flags&=14680066,E=p.alternate,E===null?(p.childLanes=0,p.lanes=n,p.child=null,p.subtreeFlags=0,p.memoizedProps=null,p.memoizedState=null,p.updateQueue=null,p.dependencies=null,p.stateNode=null):(p.childLanes=E.childLanes,p.lanes=E.lanes,p.child=E.child,p.subtreeFlags=0,p.deletions=null,p.memoizedProps=E.memoizedProps,p.memoizedState=E.memoizedState,p.updateQueue=E.updateQueue,p.type=E.type,n=E.dependencies,p.dependencies=n===null?null:{lanes:n.lanes,firstContext:n.firstContext}),a=a.sibling;return Dt(zt,zt.current&1|2),i.child}n=n.sibling}p.tail!==null&&Re()>ws&&(i.flags|=128,l=!0,Ua(p,!1),i.lanes=4194304)}else{if(!l)if(n=Fo(E),n!==null){if(i.flags|=128,l=!0,a=n.updateQueue,a!==null&&(i.updateQueue=a,i.flags|=4),Ua(p,!0),p.tail===null&&p.tailMode==="hidden"&&!E.alternate&&!Bt)return dn(i),null}else 2*Re()-p.renderingStartTime>ws&&a!==1073741824&&(i.flags|=128,l=!0,Ua(p,!1),i.lanes=4194304);p.isBackwards?(E.sibling=i.child,i.child=E):(a=p.last,a!==null?a.sibling=E:i.child=E,p.last=E)}return p.tail!==null?(i=p.tail,p.rendering=i,p.tail=i.sibling,p.renderingStartTime=Re(),i.sibling=null,a=zt.current,Dt(zt,l?a&1|2:a&1),i):(dn(i),null);case 22:case 23:return Lc(),l=i.memoizedState!==null,n!==null&&n.memoizedState!==null!==l&&(i.flags|=8192),l&&(i.mode&1)!==0?(On&1073741824)!==0&&(dn(i),i.subtreeFlags&6&&(i.flags|=8192)):dn(i),null;case 24:return null;case 25:return null}throw Error(t(156,i.tag))}function Ev(n,i){switch(zu(i),i.tag){case 1:return An(i.type)&&To(),n=i.flags,n&65536?(i.flags=n&-65537|128,i):null;case 3:return ys(),Ft(Tn),Ft(cn),Ju(),n=i.flags,(n&65536)!==0&&(n&128)===0?(i.flags=n&-65537|128,i):null;case 5:return Zu(i),null;case 13:if(Ft(zt),n=i.memoizedState,n!==null&&n.dehydrated!==null){if(i.alternate===null)throw Error(t(340));gs()}return n=i.flags,n&65536?(i.flags=n&-65537|128,i):null;case 19:return Ft(zt),null;case 4:return ys(),null;case 10:return ju(i.type._context),null;case 22:case 23:return Lc(),null;case 24:return null;default:return null}}var Xo=!1,hn=!1,wv=typeof WeakSet=="function"?WeakSet:Set,Be=null;function Ms(n,i){var a=n.ref;if(a!==null)if(typeof a=="function")try{a(null)}catch(l){Gt(n,i,l)}else a.current=null}function xc(n,i,a){try{a()}catch(l){Gt(n,i,l)}}var bp=!1;function Tv(n,i){if(Lu=co,n=oh(),Eu(n)){if("selectionStart"in n)var a={start:n.selectionStart,end:n.selectionEnd};else e:{a=(a=n.ownerDocument)&&a.defaultView||window;var l=a.getSelection&&a.getSelection();if(l&&l.rangeCount!==0){a=l.anchorNode;var f=l.anchorOffset,p=l.focusNode;l=l.focusOffset;try{a.nodeType,p.nodeType}catch{a=null;break e}var E=0,N=-1,H=-1,re=0,ve=0,Se=n,ge=null;t:for(;;){for(var Fe;Se!==a||f!==0&&Se.nodeType!==3||(N=E+f),Se!==p||l!==0&&Se.nodeType!==3||(H=E+l),Se.nodeType===3&&(E+=Se.nodeValue.length),(Fe=Se.firstChild)!==null;)ge=Se,Se=Fe;for(;;){if(Se===n)break t;if(ge===a&&++re===f&&(N=E),ge===p&&++ve===l&&(H=E),(Fe=Se.nextSibling)!==null)break;Se=ge,ge=Se.parentNode}Se=Fe}a=N===-1||H===-1?null:{start:N,end:H}}else a=null}a=a||{start:0,end:0}}else a=null;for(Nu={focusedElem:n,selectionRange:a},co=!1,Be=i;Be!==null;)if(i=Be,n=i.child,(i.subtreeFlags&1028)!==0&&n!==null)n.return=i,Be=n;else for(;Be!==null;){i=Be;try{var ze=i.alternate;if((i.flags&1024)!==0)switch(i.tag){case 0:case 11:case 15:break;case 1:if(ze!==null){var Ve=ze.memoizedProps,Wt=ze.memoizedState,Q=i.stateNode,W=Q.getSnapshotBeforeUpdate(i.elementType===i.type?Ve:ti(i.type,Ve),Wt);Q.__reactInternalSnapshotBeforeUpdate=W}break;case 3:var ee=i.stateNode.containerInfo;ee.nodeType===1?ee.textContent="":ee.nodeType===9&&ee.documentElement&&ee.removeChild(ee.documentElement);break;case 5:case 6:case 4:case 17:break;default:throw Error(t(163))}}catch(Ae){Gt(i,i.return,Ae)}if(n=i.sibling,n!==null){n.return=i.return,Be=n;break}Be=i.return}return ze=bp,bp=!1,ze}function Fa(n,i,a){var l=i.updateQueue;if(l=l!==null?l.lastEffect:null,l!==null){var f=l=l.next;do{if((f.tag&n)===n){var p=f.destroy;f.destroy=void 0,p!==void 0&&xc(i,a,p)}f=f.next}while(f!==l)}}function jo(n,i){if(i=i.updateQueue,i=i!==null?i.lastEffect:null,i!==null){var a=i=i.next;do{if((a.tag&n)===n){var l=a.create;a.destroy=l()}a=a.next}while(a!==i)}}function yc(n){var i=n.ref;if(i!==null){var a=n.stateNode;switch(n.tag){case 5:n=a;break;default:n=a}typeof i=="function"?i(n):i.current=n}}function Pp(n){var i=n.alternate;i!==null&&(n.alternate=null,Pp(i)),n.child=null,n.deletions=null,n.sibling=null,n.tag===5&&(i=n.stateNode,i!==null&&(delete i[hi],delete i[Ta],delete i[Fu],delete i[ov],delete i[lv])),n.stateNode=null,n.return=null,n.dependencies=null,n.memoizedProps=null,n.memoizedState=null,n.pendingProps=null,n.stateNode=null,n.updateQueue=null}function Lp(n){return n.tag===5||n.tag===3||n.tag===4}function Np(n){e:for(;;){for(;n.sibling===null;){if(n.return===null||Lp(n.return))return null;n=n.return}for(n.sibling.return=n.return,n=n.sibling;n.tag!==5&&n.tag!==6&&n.tag!==18;){if(n.flags&2||n.child===null||n.tag===4)continue e;n.child.return=n,n=n.child}if(!(n.flags&2))return n.stateNode}}function Sc(n,i,a){var l=n.tag;if(l===5||l===6)n=n.stateNode,i?a.nodeType===8?a.parentNode.insertBefore(n,i):a.insertBefore(n,i):(a.nodeType===8?(i=a.parentNode,i.insertBefore(n,a)):(i=a,i.appendChild(n)),a=a._reactRootContainer,a!=null||i.onclick!==null||(i.onclick=Eo));else if(l!==4&&(n=n.child,n!==null))for(Sc(n,i,a),n=n.sibling;n!==null;)Sc(n,i,a),n=n.sibling}function Mc(n,i,a){var l=n.tag;if(l===5||l===6)n=n.stateNode,i?a.insertBefore(n,i):a.appendChild(n);else if(l!==4&&(n=n.child,n!==null))for(Mc(n,i,a),n=n.sibling;n!==null;)Mc(n,i,a),n=n.sibling}var ln=null,ni=!1;function sr(n,i,a){for(a=a.child;a!==null;)Dp(n,i,a),a=a.sibling}function Dp(n,i,a){if(ct&&typeof ct.onCommitFiberUnmount=="function")try{ct.onCommitFiberUnmount(tn,a)}catch{}switch(a.tag){case 5:hn||Ms(a,i);case 6:var l=ln,f=ni;ln=null,sr(n,i,a),ln=l,ni=f,ln!==null&&(ni?(n=ln,a=a.stateNode,n.nodeType===8?n.parentNode.removeChild(a):n.removeChild(a)):ln.removeChild(a.stateNode));break;case 18:ln!==null&&(ni?(n=ln,a=a.stateNode,n.nodeType===8?Uu(n.parentNode,a):n.nodeType===1&&Uu(n,a),pa(n)):Uu(ln,a.stateNode));break;case 4:l=ln,f=ni,ln=a.stateNode.containerInfo,ni=!0,sr(n,i,a),ln=l,ni=f;break;case 0:case 11:case 14:case 15:if(!hn&&(l=a.updateQueue,l!==null&&(l=l.lastEffect,l!==null))){f=l=l.next;do{var p=f,E=p.destroy;p=p.tag,E!==void 0&&((p&2)!==0||(p&4)!==0)&&xc(a,i,E),f=f.next}while(f!==l)}sr(n,i,a);break;case 1:if(!hn&&(Ms(a,i),l=a.stateNode,typeof l.componentWillUnmount=="function"))try{l.props=a.memoizedProps,l.state=a.memoizedState,l.componentWillUnmount()}catch(N){Gt(a,i,N)}sr(n,i,a);break;case 21:sr(n,i,a);break;case 22:a.mode&1?(hn=(l=hn)||a.memoizedState!==null,sr(n,i,a),hn=l):sr(n,i,a);break;default:sr(n,i,a)}}function Ip(n){var i=n.updateQueue;if(i!==null){n.updateQueue=null;var a=n.stateNode;a===null&&(a=n.stateNode=new wv),i.forEach(function(l){var f=Iv.bind(null,n,l);a.has(l)||(a.add(l),l.then(f,f))})}}function ii(n,i){var a=i.deletions;if(a!==null)for(var l=0;l<a.length;l++){var f=a[l];try{var p=n,E=i,N=E;e:for(;N!==null;){switch(N.tag){case 5:ln=N.stateNode,ni=!1;break e;case 3:ln=N.stateNode.containerInfo,ni=!0;break e;case 4:ln=N.stateNode.containerInfo,ni=!0;break e}N=N.return}if(ln===null)throw Error(t(160));Dp(p,E,f),ln=null,ni=!1;var H=f.alternate;H!==null&&(H.return=null),f.return=null}catch(re){Gt(f,i,re)}}if(i.subtreeFlags&12854)for(i=i.child;i!==null;)Up(i,n),i=i.sibling}function Up(n,i){var a=n.alternate,l=n.flags;switch(n.tag){case 0:case 11:case 14:case 15:if(ii(i,n),gi(n),l&4){try{Fa(3,n,n.return),jo(3,n)}catch(Ve){Gt(n,n.return,Ve)}try{Fa(5,n,n.return)}catch(Ve){Gt(n,n.return,Ve)}}break;case 1:ii(i,n),gi(n),l&512&&a!==null&&Ms(a,a.return);break;case 5:if(ii(i,n),gi(n),l&512&&a!==null&&Ms(a,a.return),n.flags&32){var f=n.stateNode;try{Ee(f,"")}catch(Ve){Gt(n,n.return,Ve)}}if(l&4&&(f=n.stateNode,f!=null)){var p=n.memoizedProps,E=a!==null?a.memoizedProps:p,N=n.type,H=n.updateQueue;if(n.updateQueue=null,H!==null)try{N==="input"&&p.type==="radio"&&p.name!=null&&We(f,p),Tt(N,E);var re=Tt(N,p);for(E=0;E<H.length;E+=2){var ve=H[E],Se=H[E+1];ve==="style"?Oe(f,Se):ve==="dangerouslySetInnerHTML"?ot(f,Se):ve==="children"?Ee(f,Se):R(f,ve,Se,re)}switch(N){case"input":Lt(f,p);break;case"textarea":ye(f,p);break;case"select":var ge=f._wrapperState.wasMultiple;f._wrapperState.wasMultiple=!!p.multiple;var Fe=p.value;Fe!=null?A(f,!!p.multiple,Fe,!1):ge!==!!p.multiple&&(p.defaultValue!=null?A(f,!!p.multiple,p.defaultValue,!0):A(f,!!p.multiple,p.multiple?[]:"",!1))}f[Ta]=p}catch(Ve){Gt(n,n.return,Ve)}}break;case 6:if(ii(i,n),gi(n),l&4){if(n.stateNode===null)throw Error(t(162));f=n.stateNode,p=n.memoizedProps;try{f.nodeValue=p}catch(Ve){Gt(n,n.return,Ve)}}break;case 3:if(ii(i,n),gi(n),l&4&&a!==null&&a.memoizedState.isDehydrated)try{pa(i.containerInfo)}catch(Ve){Gt(n,n.return,Ve)}break;case 4:ii(i,n),gi(n);break;case 13:ii(i,n),gi(n),f=n.child,f.flags&8192&&(p=f.memoizedState!==null,f.stateNode.isHidden=p,!p||f.alternate!==null&&f.alternate.memoizedState!==null||(Tc=Re())),l&4&&Ip(n);break;case 22:if(ve=a!==null&&a.memoizedState!==null,n.mode&1?(hn=(re=hn)||ve,ii(i,n),hn=re):ii(i,n),gi(n),l&8192){if(re=n.memoizedState!==null,(n.stateNode.isHidden=re)&&!ve&&(n.mode&1)!==0)for(Be=n,ve=n.child;ve!==null;){for(Se=Be=ve;Be!==null;){switch(ge=Be,Fe=ge.child,ge.tag){case 0:case 11:case 14:case 15:Fa(4,ge,ge.return);break;case 1:Ms(ge,ge.return);var ze=ge.stateNode;if(typeof ze.componentWillUnmount=="function"){l=ge,a=ge.return;try{i=l,ze.props=i.memoizedProps,ze.state=i.memoizedState,ze.componentWillUnmount()}catch(Ve){Gt(l,a,Ve)}}break;case 5:Ms(ge,ge.return);break;case 22:if(ge.memoizedState!==null){kp(Se);continue}}Fe!==null?(Fe.return=ge,Be=Fe):kp(Se)}ve=ve.sibling}e:for(ve=null,Se=n;;){if(Se.tag===5){if(ve===null){ve=Se;try{f=Se.stateNode,re?(p=f.style,typeof p.setProperty=="function"?p.setProperty("display","none","important"):p.display="none"):(N=Se.stateNode,H=Se.memoizedProps.style,E=H!=null&&H.hasOwnProperty("display")?H.display:null,N.style.display=Je("display",E))}catch(Ve){Gt(n,n.return,Ve)}}}else if(Se.tag===6){if(ve===null)try{Se.stateNode.nodeValue=re?"":Se.memoizedProps}catch(Ve){Gt(n,n.return,Ve)}}else if((Se.tag!==22&&Se.tag!==23||Se.memoizedState===null||Se===n)&&Se.child!==null){Se.child.return=Se,Se=Se.child;continue}if(Se===n)break e;for(;Se.sibling===null;){if(Se.return===null||Se.return===n)break e;ve===Se&&(ve=null),Se=Se.return}ve===Se&&(ve=null),Se.sibling.return=Se.return,Se=Se.sibling}}break;case 19:ii(i,n),gi(n),l&4&&Ip(n);break;case 21:break;default:ii(i,n),gi(n)}}function gi(n){var i=n.flags;if(i&2){try{e:{for(var a=n.return;a!==null;){if(Lp(a)){var l=a;break e}a=a.return}throw Error(t(160))}switch(l.tag){case 5:var f=l.stateNode;l.flags&32&&(Ee(f,""),l.flags&=-33);var p=Np(n);Mc(n,p,f);break;case 3:case 4:var E=l.stateNode.containerInfo,N=Np(n);Sc(n,N,E);break;default:throw Error(t(161))}}catch(H){Gt(n,n.return,H)}n.flags&=-3}i&4096&&(n.flags&=-4097)}function Av(n,i,a){Be=n,Fp(n)}function Fp(n,i,a){for(var l=(n.mode&1)!==0;Be!==null;){var f=Be,p=f.child;if(f.tag===22&&l){var E=f.memoizedState!==null||Xo;if(!E){var N=f.alternate,H=N!==null&&N.memoizedState!==null||hn;N=Xo;var re=hn;if(Xo=E,(hn=H)&&!re)for(Be=f;Be!==null;)E=Be,H=E.child,E.tag===22&&E.memoizedState!==null?Bp(f):H!==null?(H.return=E,Be=H):Bp(f);for(;p!==null;)Be=p,Fp(p),p=p.sibling;Be=f,Xo=N,hn=re}Op(n)}else(f.subtreeFlags&8772)!==0&&p!==null?(p.return=f,Be=p):Op(n)}}function Op(n){for(;Be!==null;){var i=Be;if((i.flags&8772)!==0){var a=i.alternate;try{if((i.flags&8772)!==0)switch(i.tag){case 0:case 11:case 15:hn||jo(5,i);break;case 1:var l=i.stateNode;if(i.flags&4&&!hn)if(a===null)l.componentDidMount();else{var f=i.elementType===i.type?a.memoizedProps:ti(i.type,a.memoizedProps);l.componentDidUpdate(f,a.memoizedState,l.__reactInternalSnapshotBeforeUpdate)}var p=i.updateQueue;p!==null&&kh(i,p,l);break;case 3:var E=i.updateQueue;if(E!==null){if(a=null,i.child!==null)switch(i.child.tag){case 5:a=i.child.stateNode;break;case 1:a=i.child.stateNode}kh(i,E,a)}break;case 5:var N=i.stateNode;if(a===null&&i.flags&4){a=N;var H=i.memoizedProps;switch(i.type){case"button":case"input":case"select":case"textarea":H.autoFocus&&a.focus();break;case"img":H.src&&(a.src=H.src)}}break;case 6:break;case 4:break;case 12:break;case 13:if(i.memoizedState===null){var re=i.alternate;if(re!==null){var ve=re.memoizedState;if(ve!==null){var Se=ve.dehydrated;Se!==null&&pa(Se)}}}break;case 19:case 17:case 21:case 22:case 23:case 25:break;default:throw Error(t(163))}hn||i.flags&512&&yc(i)}catch(ge){Gt(i,i.return,ge)}}if(i===n){Be=null;break}if(a=i.sibling,a!==null){a.return=i.return,Be=a;break}Be=i.return}}function kp(n){for(;Be!==null;){var i=Be;if(i===n){Be=null;break}var a=i.sibling;if(a!==null){a.return=i.return,Be=a;break}Be=i.return}}function Bp(n){for(;Be!==null;){var i=Be;try{switch(i.tag){case 0:case 11:case 15:var a=i.return;try{jo(4,i)}catch(H){Gt(i,a,H)}break;case 1:var l=i.stateNode;if(typeof l.componentDidMount=="function"){var f=i.return;try{l.componentDidMount()}catch(H){Gt(i,f,H)}}var p=i.return;try{yc(i)}catch(H){Gt(i,p,H)}break;case 5:var E=i.return;try{yc(i)}catch(H){Gt(i,E,H)}}}catch(H){Gt(i,i.return,H)}if(i===n){Be=null;break}var N=i.sibling;if(N!==null){N.return=i.return,Be=N;break}Be=i.return}}var Cv=Math.ceil,Yo=L.ReactCurrentDispatcher,Ec=L.ReactCurrentOwner,jn=L.ReactCurrentBatchConfig,St=0,sn=null,Xt=null,un=0,On=0,Es=er(0),Kt=0,Oa=null,Ir=0,qo=0,wc=0,ka=null,Rn=null,Tc=0,ws=1/0,Li=null,$o=!1,Ac=null,ar=null,Ko=!1,or=null,Zo=0,Ba=0,Cc=null,Qo=-1,Jo=0;function xn(){return(St&6)!==0?Re():Qo!==-1?Qo:Qo=Re()}function lr(n){return(n.mode&1)===0?1:(St&2)!==0&&un!==0?un&-un:cv.transition!==null?(Jo===0&&(Jo=oo()),Jo):(n=Rt,n!==0||(n=window.event,n=n===void 0?16:Hd(n.type)),n)}function ri(n,i,a,l){if(50<Ba)throw Ba=0,Cc=null,Error(t(185));ua(n,a,l),((St&2)===0||n!==sn)&&(n===sn&&((St&2)===0&&(qo|=a),Kt===4&&ur(n,un)),bn(n,l),a===1&&St===0&&(i.mode&1)===0&&(ws=Re()+500,Co&&nr()))}function bn(n,i){var a=n.callbackNode;wn(n,i);var l=Dn(n,n===sn?un:0);if(l===0)a!==null&&Ce(a),n.callbackNode=null,n.callbackPriority=0;else if(i=l&-l,n.callbackPriority!==i){if(a!=null&&Ce(a),i===1)n.tag===0?uv(Hp.bind(null,n)):Ah(Hp.bind(null,n)),sv(function(){(St&6)===0&&nr()}),a=null;else{switch(Dd(l)){case 1:a=et;break;case 4:a=$e;break;case 16:a=xt;break;case 536870912:a=It;break;default:a=xt}a=$p(a,zp.bind(null,n))}n.callbackPriority=i,n.callbackNode=a}}function zp(n,i){if(Qo=-1,Jo=0,(St&6)!==0)throw Error(t(327));var a=n.callbackNode;if(Ts()&&n.callbackNode!==a)return null;var l=Dn(n,n===sn?un:0);if(l===0)return null;if((l&30)!==0||(l&n.expiredLanes)!==0||i)i=el(n,l);else{i=l;var f=St;St|=2;var p=Gp();(sn!==n||un!==i)&&(Li=null,ws=Re()+500,Fr(n,i));do try{Pv();break}catch(N){Vp(n,N)}while(!0);Xu(),Yo.current=p,St=f,Xt!==null?i=0:(sn=null,un=0,i=Kt)}if(i!==0){if(i===2&&(f=Ar(n),f!==0&&(l=f,i=Rc(n,f))),i===1)throw a=Oa,Fr(n,0),ur(n,l),bn(n,Re()),a;if(i===6)ur(n,l);else{if(f=n.current.alternate,(l&30)===0&&!Rv(f)&&(i=el(n,l),i===2&&(p=Ar(n),p!==0&&(l=p,i=Rc(n,p))),i===1))throw a=Oa,Fr(n,0),ur(n,l),bn(n,Re()),a;switch(n.finishedWork=f,n.finishedLanes=l,i){case 0:case 1:throw Error(t(345));case 2:Or(n,Rn,Li);break;case 3:if(ur(n,l),(l&130023424)===l&&(i=Tc+500-Re(),10<i)){if(Dn(n,0)!==0)break;if(f=n.suspendedLanes,(f&l)!==l){xn(),n.pingedLanes|=n.suspendedLanes&f;break}n.timeoutHandle=Iu(Or.bind(null,n,Rn,Li),i);break}Or(n,Rn,Li);break;case 4:if(ur(n,l),(l&4194240)===l)break;for(i=n.eventTimes,f=-1;0<l;){var E=31-Ct(l);p=1<<E,E=i[E],E>f&&(f=E),l&=~p}if(l=f,l=Re()-l,l=(120>l?120:480>l?480:1080>l?1080:1920>l?1920:3e3>l?3e3:4320>l?4320:1960*Cv(l/1960))-l,10<l){n.timeoutHandle=Iu(Or.bind(null,n,Rn,Li),l);break}Or(n,Rn,Li);break;case 5:Or(n,Rn,Li);break;default:throw Error(t(329))}}}return bn(n,Re()),n.callbackNode===a?zp.bind(null,n):null}function Rc(n,i){var a=ka;return n.current.memoizedState.isDehydrated&&(Fr(n,i).flags|=256),n=el(n,i),n!==2&&(i=Rn,Rn=a,i!==null&&bc(i)),n}function bc(n){Rn===null?Rn=n:Rn.push.apply(Rn,n)}function Rv(n){for(var i=n;;){if(i.flags&16384){var a=i.updateQueue;if(a!==null&&(a=a.stores,a!==null))for(var l=0;l<a.length;l++){var f=a[l],p=f.getSnapshot;f=f.value;try{if(!Jn(p(),f))return!1}catch{return!1}}}if(a=i.child,i.subtreeFlags&16384&&a!==null)a.return=i,i=a;else{if(i===n)break;for(;i.sibling===null;){if(i.return===null||i.return===n)return!0;i=i.return}i.sibling.return=i.return,i=i.sibling}}return!0}function ur(n,i){for(i&=~wc,i&=~qo,n.suspendedLanes|=i,n.pingedLanes&=~i,n=n.expirationTimes;0<i;){var a=31-Ct(i),l=1<<a;n[a]=-1,i&=~l}}function Hp(n){if((St&6)!==0)throw Error(t(327));Ts();var i=Dn(n,0);if((i&1)===0)return bn(n,Re()),null;var a=el(n,i);if(n.tag!==0&&a===2){var l=Ar(n);l!==0&&(i=l,a=Rc(n,l))}if(a===1)throw a=Oa,Fr(n,0),ur(n,i),bn(n,Re()),a;if(a===6)throw Error(t(345));return n.finishedWork=n.current.alternate,n.finishedLanes=i,Or(n,Rn,Li),bn(n,Re()),null}function Pc(n,i){var a=St;St|=1;try{return n(i)}finally{St=a,St===0&&(ws=Re()+500,Co&&nr())}}function Ur(n){or!==null&&or.tag===0&&(St&6)===0&&Ts();var i=St;St|=1;var a=jn.transition,l=Rt;try{if(jn.transition=null,Rt=1,n)return n()}finally{Rt=l,jn.transition=a,St=i,(St&6)===0&&nr()}}function Lc(){On=Es.current,Ft(Es)}function Fr(n,i){n.finishedWork=null,n.finishedLanes=0;var a=n.timeoutHandle;if(a!==-1&&(n.timeoutHandle=-1,rv(a)),Xt!==null)for(a=Xt.return;a!==null;){var l=a;switch(zu(l),l.tag){case 1:l=l.type.childContextTypes,l!=null&&To();break;case 3:ys(),Ft(Tn),Ft(cn),Ju();break;case 5:Zu(l);break;case 4:ys();break;case 13:Ft(zt);break;case 19:Ft(zt);break;case 10:ju(l.type._context);break;case 22:case 23:Lc()}a=a.return}if(sn=n,Xt=n=cr(n.current,null),un=On=i,Kt=0,Oa=null,wc=qo=Ir=0,Rn=ka=null,Lr!==null){for(i=0;i<Lr.length;i++)if(a=Lr[i],l=a.interleaved,l!==null){a.interleaved=null;var f=l.next,p=a.pending;if(p!==null){var E=p.next;p.next=f,l.next=E}a.pending=l}Lr=null}return n}function Vp(n,i){do{var a=Xt;try{if(Xu(),Oo.current=Ho,ko){for(var l=Ht.memoizedState;l!==null;){var f=l.queue;f!==null&&(f.pending=null),l=l.next}ko=!1}if(Dr=0,rn=$t=Ht=null,La=!1,Na=0,Ec.current=null,a===null||a.return===null){Kt=1,Oa=i,Xt=null;break}e:{var p=n,E=a.return,N=a,H=i;if(i=un,N.flags|=32768,H!==null&&typeof H=="object"&&typeof H.then=="function"){var re=H,ve=N,Se=ve.tag;if((ve.mode&1)===0&&(Se===0||Se===11||Se===15)){var ge=ve.alternate;ge?(ve.updateQueue=ge.updateQueue,ve.memoizedState=ge.memoizedState,ve.lanes=ge.lanes):(ve.updateQueue=null,ve.memoizedState=null)}var Fe=hp(E);if(Fe!==null){Fe.flags&=-257,pp(Fe,E,N,p,i),Fe.mode&1&&dp(p,re,i),i=Fe,H=re;var ze=i.updateQueue;if(ze===null){var Ve=new Set;Ve.add(H),i.updateQueue=Ve}else ze.add(H);break e}else{if((i&1)===0){dp(p,re,i),Nc();break e}H=Error(t(426))}}else if(Bt&&N.mode&1){var Wt=hp(E);if(Wt!==null){(Wt.flags&65536)===0&&(Wt.flags|=256),pp(Wt,E,N,p,i),Gu(Ss(H,N));break e}}p=H=Ss(H,N),Kt!==4&&(Kt=2),ka===null?ka=[p]:ka.push(p),p=E;do{switch(p.tag){case 3:p.flags|=65536,i&=-i,p.lanes|=i;var Q=cp(p,H,i);Oh(p,Q);break e;case 1:N=H;var W=p.type,ee=p.stateNode;if((p.flags&128)===0&&(typeof W.getDerivedStateFromError=="function"||ee!==null&&typeof ee.componentDidCatch=="function"&&(ar===null||!ar.has(ee)))){p.flags|=65536,i&=-i,p.lanes|=i;var Ae=fp(p,N,i);Oh(p,Ae);break e}}p=p.return}while(p!==null)}Xp(a)}catch(Ge){i=Ge,Xt===a&&a!==null&&(Xt=a=a.return);continue}break}while(!0)}function Gp(){var n=Yo.current;return Yo.current=Ho,n===null?Ho:n}function Nc(){(Kt===0||Kt===3||Kt===2)&&(Kt=4),sn===null||(Ir&268435455)===0&&(qo&268435455)===0||ur(sn,un)}function el(n,i){var a=St;St|=2;var l=Gp();(sn!==n||un!==i)&&(Li=null,Fr(n,i));do try{bv();break}catch(f){Vp(n,f)}while(!0);if(Xu(),St=a,Yo.current=l,Xt!==null)throw Error(t(261));return sn=null,un=0,Kt}function bv(){for(;Xt!==null;)Wp(Xt)}function Pv(){for(;Xt!==null&&!Ue();)Wp(Xt)}function Wp(n){var i=qp(n.alternate,n,On);n.memoizedProps=n.pendingProps,i===null?Xp(n):Xt=i,Ec.current=null}function Xp(n){var i=n;do{var a=i.alternate;if(n=i.return,(i.flags&32768)===0){if(a=Mv(a,i,On),a!==null){Xt=a;return}}else{if(a=Ev(a,i),a!==null){a.flags&=32767,Xt=a;return}if(n!==null)n.flags|=32768,n.subtreeFlags=0,n.deletions=null;else{Kt=6,Xt=null;return}}if(i=i.sibling,i!==null){Xt=i;return}Xt=i=n}while(i!==null);Kt===0&&(Kt=5)}function Or(n,i,a){var l=Rt,f=jn.transition;try{jn.transition=null,Rt=1,Lv(n,i,a,l)}finally{jn.transition=f,Rt=l}return null}function Lv(n,i,a,l){do Ts();while(or!==null);if((St&6)!==0)throw Error(t(327));a=n.finishedWork;var f=n.finishedLanes;if(a===null)return null;if(n.finishedWork=null,n.finishedLanes=0,a===n.current)throw Error(t(177));n.callbackNode=null,n.callbackPriority=0;var p=a.lanes|a.childLanes;if(f_(n,p),n===sn&&(Xt=sn=null,un=0),(a.subtreeFlags&2064)===0&&(a.flags&2064)===0||Ko||(Ko=!0,$p(xt,function(){return Ts(),null})),p=(a.flags&15990)!==0,(a.subtreeFlags&15990)!==0||p){p=jn.transition,jn.transition=null;var E=Rt;Rt=1;var N=St;St|=4,Ec.current=null,Tv(n,a),Up(a,n),Z_(Nu),co=!!Lu,Nu=Lu=null,n.current=a,Av(a),He(),St=N,Rt=E,jn.transition=p}else n.current=a;if(Ko&&(Ko=!1,or=n,Zo=f),p=n.pendingLanes,p===0&&(ar=null),Xe(a.stateNode),bn(n,Re()),i!==null)for(l=n.onRecoverableError,a=0;a<i.length;a++)f=i[a],l(f.value,{componentStack:f.stack,digest:f.digest});if($o)throw $o=!1,n=Ac,Ac=null,n;return(Zo&1)!==0&&n.tag!==0&&Ts(),p=n.pendingLanes,(p&1)!==0?n===Cc?Ba++:(Ba=0,Cc=n):Ba=0,nr(),null}function Ts(){if(or!==null){var n=Dd(Zo),i=jn.transition,a=Rt;try{if(jn.transition=null,Rt=16>n?16:n,or===null)var l=!1;else{if(n=or,or=null,Zo=0,(St&6)!==0)throw Error(t(331));var f=St;for(St|=4,Be=n.current;Be!==null;){var p=Be,E=p.child;if((Be.flags&16)!==0){var N=p.deletions;if(N!==null){for(var H=0;H<N.length;H++){var re=N[H];for(Be=re;Be!==null;){var ve=Be;switch(ve.tag){case 0:case 11:case 15:Fa(8,ve,p)}var Se=ve.child;if(Se!==null)Se.return=ve,Be=Se;else for(;Be!==null;){ve=Be;var ge=ve.sibling,Fe=ve.return;if(Pp(ve),ve===re){Be=null;break}if(ge!==null){ge.return=Fe,Be=ge;break}Be=Fe}}}var ze=p.alternate;if(ze!==null){var Ve=ze.child;if(Ve!==null){ze.child=null;do{var Wt=Ve.sibling;Ve.sibling=null,Ve=Wt}while(Ve!==null)}}Be=p}}if((p.subtreeFlags&2064)!==0&&E!==null)E.return=p,Be=E;else e:for(;Be!==null;){if(p=Be,(p.flags&2048)!==0)switch(p.tag){case 0:case 11:case 15:Fa(9,p,p.return)}var Q=p.sibling;if(Q!==null){Q.return=p.return,Be=Q;break e}Be=p.return}}var W=n.current;for(Be=W;Be!==null;){E=Be;var ee=E.child;if((E.subtreeFlags&2064)!==0&&ee!==null)ee.return=E,Be=ee;else e:for(E=W;Be!==null;){if(N=Be,(N.flags&2048)!==0)try{switch(N.tag){case 0:case 11:case 15:jo(9,N)}}catch(Ge){Gt(N,N.return,Ge)}if(N===E){Be=null;break e}var Ae=N.sibling;if(Ae!==null){Ae.return=N.return,Be=Ae;break e}Be=N.return}}if(St=f,nr(),ct&&typeof ct.onPostCommitFiberRoot=="function")try{ct.onPostCommitFiberRoot(tn,n)}catch{}l=!0}return l}finally{Rt=a,jn.transition=i}}return!1}function jp(n,i,a){i=Ss(a,i),i=cp(n,i,1),n=rr(n,i,1),i=xn(),n!==null&&(ua(n,1,i),bn(n,i))}function Gt(n,i,a){if(n.tag===3)jp(n,n,a);else for(;i!==null;){if(i.tag===3){jp(i,n,a);break}else if(i.tag===1){var l=i.stateNode;if(typeof i.type.getDerivedStateFromError=="function"||typeof l.componentDidCatch=="function"&&(ar===null||!ar.has(l))){n=Ss(a,n),n=fp(i,n,1),i=rr(i,n,1),n=xn(),i!==null&&(ua(i,1,n),bn(i,n));break}}i=i.return}}function Nv(n,i,a){var l=n.pingCache;l!==null&&l.delete(i),i=xn(),n.pingedLanes|=n.suspendedLanes&a,sn===n&&(un&a)===a&&(Kt===4||Kt===3&&(un&130023424)===un&&500>Re()-Tc?Fr(n,0):wc|=a),bn(n,i)}function Yp(n,i){i===0&&((n.mode&1)===0?i=1:(i=ji,ji<<=1,(ji&130023424)===0&&(ji=4194304)));var a=xn();n=Ri(n,i),n!==null&&(ua(n,i,a),bn(n,a))}function Dv(n){var i=n.memoizedState,a=0;i!==null&&(a=i.retryLane),Yp(n,a)}function Iv(n,i){var a=0;switch(n.tag){case 13:var l=n.stateNode,f=n.memoizedState;f!==null&&(a=f.retryLane);break;case 19:l=n.stateNode;break;default:throw Error(t(314))}l!==null&&l.delete(i),Yp(n,a)}var qp;qp=function(n,i,a){if(n!==null)if(n.memoizedProps!==i.pendingProps||Tn.current)Cn=!0;else{if((n.lanes&a)===0&&(i.flags&128)===0)return Cn=!1,Sv(n,i,a);Cn=(n.flags&131072)!==0}else Cn=!1,Bt&&(i.flags&1048576)!==0&&Ch(i,bo,i.index);switch(i.lanes=0,i.tag){case 2:var l=i.type;Wo(n,i),n=i.pendingProps;var f=hs(i,cn.current);xs(i,a),f=nc(null,i,l,n,f,a);var p=ic();return i.flags|=1,typeof f=="object"&&f!==null&&typeof f.render=="function"&&f.$$typeof===void 0?(i.tag=1,i.memoizedState=null,i.updateQueue=null,An(l)?(p=!0,Ao(i)):p=!1,i.memoizedState=f.state!==null&&f.state!==void 0?f.state:null,$u(i),f.updater=Vo,i.stateNode=f,f._reactInternals=i,uc(i,l,n,a),i=hc(null,i,l,!0,p,a)):(i.tag=0,Bt&&p&&Bu(i),vn(null,i,f,a),i=i.child),i;case 16:l=i.elementType;e:{switch(Wo(n,i),n=i.pendingProps,f=l._init,l=f(l._payload),i.type=l,f=i.tag=Fv(l),n=ti(l,n),f){case 0:i=dc(null,i,l,n,a);break e;case 1:i=yp(null,i,l,n,a);break e;case 11:i=mp(null,i,l,n,a);break e;case 14:i=gp(null,i,l,ti(l.type,n),a);break e}throw Error(t(306,l,""))}return i;case 0:return l=i.type,f=i.pendingProps,f=i.elementType===l?f:ti(l,f),dc(n,i,l,f,a);case 1:return l=i.type,f=i.pendingProps,f=i.elementType===l?f:ti(l,f),yp(n,i,l,f,a);case 3:e:{if(Sp(i),n===null)throw Error(t(387));l=i.pendingProps,p=i.memoizedState,f=p.element,Fh(n,i),Uo(i,l,null,a);var E=i.memoizedState;if(l=E.element,p.isDehydrated)if(p={element:l,isDehydrated:!1,cache:E.cache,pendingSuspenseBoundaries:E.pendingSuspenseBoundaries,transitions:E.transitions},i.updateQueue.baseState=p,i.memoizedState=p,i.flags&256){f=Ss(Error(t(423)),i),i=Mp(n,i,l,a,f);break e}else if(l!==f){f=Ss(Error(t(424)),i),i=Mp(n,i,l,a,f);break e}else for(Fn=Ji(i.stateNode.containerInfo.firstChild),Un=i,Bt=!0,ei=null,a=Ih(i,null,l,a),i.child=a;a;)a.flags=a.flags&-3|4096,a=a.sibling;else{if(gs(),l===f){i=Pi(n,i,a);break e}vn(n,i,l,a)}i=i.child}return i;case 5:return Bh(i),n===null&&Vu(i),l=i.type,f=i.pendingProps,p=n!==null?n.memoizedProps:null,E=f.children,Du(l,f)?E=null:p!==null&&Du(l,p)&&(i.flags|=32),xp(n,i),vn(n,i,E,a),i.child;case 6:return n===null&&Vu(i),null;case 13:return Ep(n,i,a);case 4:return Ku(i,i.stateNode.containerInfo),l=i.pendingProps,n===null?i.child=_s(i,null,l,a):vn(n,i,l,a),i.child;case 11:return l=i.type,f=i.pendingProps,f=i.elementType===l?f:ti(l,f),mp(n,i,l,f,a);case 7:return vn(n,i,i.pendingProps,a),i.child;case 8:return vn(n,i,i.pendingProps.children,a),i.child;case 12:return vn(n,i,i.pendingProps.children,a),i.child;case 10:e:{if(l=i.type._context,f=i.pendingProps,p=i.memoizedProps,E=f.value,Dt(No,l._currentValue),l._currentValue=E,p!==null)if(Jn(p.value,E)){if(p.children===f.children&&!Tn.current){i=Pi(n,i,a);break e}}else for(p=i.child,p!==null&&(p.return=i);p!==null;){var N=p.dependencies;if(N!==null){E=p.child;for(var H=N.firstContext;H!==null;){if(H.context===l){if(p.tag===1){H=bi(-1,a&-a),H.tag=2;var re=p.updateQueue;if(re!==null){re=re.shared;var ve=re.pending;ve===null?H.next=H:(H.next=ve.next,ve.next=H),re.pending=H}}p.lanes|=a,H=p.alternate,H!==null&&(H.lanes|=a),Yu(p.return,a,i),N.lanes|=a;break}H=H.next}}else if(p.tag===10)E=p.type===i.type?null:p.child;else if(p.tag===18){if(E=p.return,E===null)throw Error(t(341));E.lanes|=a,N=E.alternate,N!==null&&(N.lanes|=a),Yu(E,a,i),E=p.sibling}else E=p.child;if(E!==null)E.return=p;else for(E=p;E!==null;){if(E===i){E=null;break}if(p=E.sibling,p!==null){p.return=E.return,E=p;break}E=E.return}p=E}vn(n,i,f.children,a),i=i.child}return i;case 9:return f=i.type,l=i.pendingProps.children,xs(i,a),f=Wn(f),l=l(f),i.flags|=1,vn(n,i,l,a),i.child;case 14:return l=i.type,f=ti(l,i.pendingProps),f=ti(l.type,f),gp(n,i,l,f,a);case 15:return _p(n,i,i.type,i.pendingProps,a);case 17:return l=i.type,f=i.pendingProps,f=i.elementType===l?f:ti(l,f),Wo(n,i),i.tag=1,An(l)?(n=!0,Ao(i)):n=!1,xs(i,a),lp(i,l,f),uc(i,l,f,a),hc(null,i,l,!0,n,a);case 19:return Tp(n,i,a);case 22:return vp(n,i,a)}throw Error(t(156,i.tag))};function $p(n,i){return Z(n,i)}function Uv(n,i,a,l){this.tag=n,this.key=a,this.sibling=this.child=this.return=this.stateNode=this.type=this.elementType=null,this.index=0,this.ref=null,this.pendingProps=i,this.dependencies=this.memoizedState=this.updateQueue=this.memoizedProps=null,this.mode=l,this.subtreeFlags=this.flags=0,this.deletions=null,this.childLanes=this.lanes=0,this.alternate=null}function Yn(n,i,a,l){return new Uv(n,i,a,l)}function Dc(n){return n=n.prototype,!(!n||!n.isReactComponent)}function Fv(n){if(typeof n=="function")return Dc(n)?1:0;if(n!=null){if(n=n.$$typeof,n===Y)return 11;if(n===ne)return 14}return 2}function cr(n,i){var a=n.alternate;return a===null?(a=Yn(n.tag,i,n.key,n.mode),a.elementType=n.elementType,a.type=n.type,a.stateNode=n.stateNode,a.alternate=n,n.alternate=a):(a.pendingProps=i,a.type=n.type,a.flags=0,a.subtreeFlags=0,a.deletions=null),a.flags=n.flags&14680064,a.childLanes=n.childLanes,a.lanes=n.lanes,a.child=n.child,a.memoizedProps=n.memoizedProps,a.memoizedState=n.memoizedState,a.updateQueue=n.updateQueue,i=n.dependencies,a.dependencies=i===null?null:{lanes:i.lanes,firstContext:i.firstContext},a.sibling=n.sibling,a.index=n.index,a.ref=n.ref,a}function tl(n,i,a,l,f,p){var E=2;if(l=n,typeof n=="function")Dc(n)&&(E=1);else if(typeof n=="string")E=5;else e:switch(n){case D:return kr(a.children,f,p,i);case j:E=8,f|=8;break;case b:return n=Yn(12,a,i,f|2),n.elementType=b,n.lanes=p,n;case K:return n=Yn(13,a,i,f),n.elementType=K,n.lanes=p,n;case oe:return n=Yn(19,a,i,f),n.elementType=oe,n.lanes=p,n;case G:return nl(a,f,p,i);default:if(typeof n=="object"&&n!==null)switch(n.$$typeof){case w:E=10;break e;case I:E=9;break e;case Y:E=11;break e;case ne:E=14;break e;case B:E=16,l=null;break e}throw Error(t(130,n==null?n:typeof n,""))}return i=Yn(E,a,i,f),i.elementType=n,i.type=l,i.lanes=p,i}function kr(n,i,a,l){return n=Yn(7,n,l,i),n.lanes=a,n}function nl(n,i,a,l){return n=Yn(22,n,l,i),n.elementType=G,n.lanes=a,n.stateNode={isHidden:!1},n}function Ic(n,i,a){return n=Yn(6,n,null,i),n.lanes=a,n}function Uc(n,i,a){return i=Yn(4,n.children!==null?n.children:[],n.key,i),i.lanes=a,i.stateNode={containerInfo:n.containerInfo,pendingChildren:null,implementation:n.implementation},i}function Ov(n,i,a,l,f){this.tag=i,this.containerInfo=n,this.finishedWork=this.pingCache=this.current=this.pendingChildren=null,this.timeoutHandle=-1,this.callbackNode=this.pendingContext=this.context=null,this.callbackPriority=0,this.eventTimes=is(0),this.expirationTimes=is(-1),this.entangledLanes=this.finishedLanes=this.mutableReadLanes=this.expiredLanes=this.pingedLanes=this.suspendedLanes=this.pendingLanes=0,this.entanglements=is(0),this.identifierPrefix=l,this.onRecoverableError=f,this.mutableSourceEagerHydrationData=null}function Fc(n,i,a,l,f,p,E,N,H){return n=new Ov(n,i,a,N,H),i===1?(i=1,p===!0&&(i|=8)):i=0,p=Yn(3,null,null,i),n.current=p,p.stateNode=n,p.memoizedState={element:l,isDehydrated:a,cache:null,transitions:null,pendingSuspenseBoundaries:null},$u(p),n}function kv(n,i,a){var l=3<arguments.length&&arguments[3]!==void 0?arguments[3]:null;return{$$typeof:O,key:l==null?null:""+l,children:n,containerInfo:i,implementation:a}}function Kp(n){if(!n)return tr;n=n._reactInternals;e:{if(Ei(n)!==n||n.tag!==1)throw Error(t(170));var i=n;do{switch(i.tag){case 3:i=i.stateNode.context;break e;case 1:if(An(i.type)){i=i.stateNode.__reactInternalMemoizedMergedChildContext;break e}}i=i.return}while(i!==null);throw Error(t(171))}if(n.tag===1){var a=n.type;if(An(a))return wh(n,a,i)}return i}function Zp(n,i,a,l,f,p,E,N,H){return n=Fc(a,l,!0,n,f,p,E,N,H),n.context=Kp(null),a=n.current,l=xn(),f=lr(a),p=bi(l,f),p.callback=i??null,rr(a,p,f),n.current.lanes=f,ua(n,f,l),bn(n,l),n}function il(n,i,a,l){var f=i.current,p=xn(),E=lr(f);return a=Kp(a),i.context===null?i.context=a:i.pendingContext=a,i=bi(p,E),i.payload={element:n},l=l===void 0?null:l,l!==null&&(i.callback=l),n=rr(f,i,E),n!==null&&(ri(n,f,E,p),Io(n,f,E)),E}function rl(n){if(n=n.current,!n.child)return null;switch(n.child.tag){case 5:return n.child.stateNode;default:return n.child.stateNode}}function Qp(n,i){if(n=n.memoizedState,n!==null&&n.dehydrated!==null){var a=n.retryLane;n.retryLane=a!==0&&a<i?a:i}}function Oc(n,i){Qp(n,i),(n=n.alternate)&&Qp(n,i)}function Bv(){return null}var Jp=typeof reportError=="function"?reportError:function(n){console.error(n)};function kc(n){this._internalRoot=n}sl.prototype.render=kc.prototype.render=function(n){var i=this._internalRoot;if(i===null)throw Error(t(409));il(n,i,null,null)},sl.prototype.unmount=kc.prototype.unmount=function(){var n=this._internalRoot;if(n!==null){this._internalRoot=null;var i=n.containerInfo;Ur(function(){il(null,n,null,null)}),i[wi]=null}};function sl(n){this._internalRoot=n}sl.prototype.unstable_scheduleHydration=function(n){if(n){var i=Fd();n={blockedOn:null,target:n,priority:i};for(var a=0;a<Ki.length&&i!==0&&i<Ki[a].priority;a++);Ki.splice(a,0,n),a===0&&Bd(n)}};function Bc(n){return!(!n||n.nodeType!==1&&n.nodeType!==9&&n.nodeType!==11)}function al(n){return!(!n||n.nodeType!==1&&n.nodeType!==9&&n.nodeType!==11&&(n.nodeType!==8||n.nodeValue!==" react-mount-point-unstable "))}function em(){}function zv(n,i,a,l,f){if(f){if(typeof l=="function"){var p=l;l=function(){var re=rl(E);p.call(re)}}var E=Zp(i,l,n,0,null,!1,!1,"",em);return n._reactRootContainer=E,n[wi]=E.current,Ea(n.nodeType===8?n.parentNode:n),Ur(),E}for(;f=n.lastChild;)n.removeChild(f);if(typeof l=="function"){var N=l;l=function(){var re=rl(H);N.call(re)}}var H=Fc(n,0,!1,null,null,!1,!1,"",em);return n._reactRootContainer=H,n[wi]=H.current,Ea(n.nodeType===8?n.parentNode:n),Ur(function(){il(i,H,a,l)}),H}function ol(n,i,a,l,f){var p=a._reactRootContainer;if(p){var E=p;if(typeof f=="function"){var N=f;f=function(){var H=rl(E);N.call(H)}}il(i,E,n,f)}else E=zv(a,i,n,f,l);return rl(E)}Id=function(n){switch(n.tag){case 3:var i=n.stateNode;if(i.current.memoizedState.isDehydrated){var a=Nt(i.pendingLanes);a!==0&&(uu(i,a|1),bn(i,Re()),(St&6)===0&&(ws=Re()+500,nr()))}break;case 13:Ur(function(){var l=Ri(n,1);if(l!==null){var f=xn();ri(l,n,1,f)}}),Oc(n,1)}},cu=function(n){if(n.tag===13){var i=Ri(n,134217728);if(i!==null){var a=xn();ri(i,n,134217728,a)}Oc(n,134217728)}},Ud=function(n){if(n.tag===13){var i=lr(n),a=Ri(n,i);if(a!==null){var l=xn();ri(a,n,i,l)}Oc(n,i)}},Fd=function(){return Rt},Od=function(n,i){var a=Rt;try{return Rt=n,i()}finally{Rt=a}},de=function(n,i,a){switch(i){case"input":if(Lt(n,a),i=a.name,a.type==="radio"&&i!=null){for(a=n;a.parentNode;)a=a.parentNode;for(a=a.querySelectorAll("input[name="+JSON.stringify(""+i)+'][type="radio"]'),i=0;i<a.length;i++){var l=a[i];if(l!==n&&l.form===n.form){var f=wo(l);if(!f)throw Error(t(90));z(l),Lt(l,f)}}}break;case"textarea":ye(n,a);break;case"select":i=a.value,i!=null&&A(n,!!a.multiple,i,!1)}},mt=Pc,ut=Ur;var Hv={usingClientEntryPoint:!1,Events:[Aa,fs,wo,pe,Ke,Pc]},za={findFiberByHostInstance:Cr,bundleType:0,version:"18.3.1",rendererPackageName:"react-dom"},Vv={bundleType:za.bundleType,version:za.version,rendererPackageName:za.rendererPackageName,rendererConfig:za.rendererConfig,overrideHookState:null,overrideHookStateDeletePath:null,overrideHookStateRenamePath:null,overrideProps:null,overridePropsDeletePath:null,overridePropsRenamePath:null,setErrorHandler:null,setSuspenseHandler:null,scheduleUpdate:null,currentDispatcherRef:L.ReactCurrentDispatcher,findHostInstanceByFiber:function(n){return n=ie(n),n===null?null:n.stateNode},findFiberByHostInstance:za.findFiberByHostInstance||Bv,findHostInstancesForRefresh:null,scheduleRefresh:null,scheduleRoot:null,setRefreshHandler:null,getCurrentFiber:null,reconcilerVersion:"18.3.1-next-f1338f8080-20240426"};if(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__<"u"){var ll=__REACT_DEVTOOLS_GLOBAL_HOOK__;if(!ll.isDisabled&&ll.supportsFiber)try{tn=ll.inject(Vv),ct=ll}catch{}}return Pn.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=Hv,Pn.createPortal=function(n,i){var a=2<arguments.length&&arguments[2]!==void 0?arguments[2]:null;if(!Bc(i))throw Error(t(200));return kv(n,i,null,a)},Pn.createRoot=function(n,i){if(!Bc(n))throw Error(t(299));var a=!1,l="",f=Jp;return i!=null&&(i.unstable_strictMode===!0&&(a=!0),i.identifierPrefix!==void 0&&(l=i.identifierPrefix),i.onRecoverableError!==void 0&&(f=i.onRecoverableError)),i=Fc(n,1,!1,null,null,a,!1,l,f),n[wi]=i.current,Ea(n.nodeType===8?n.parentNode:n),new kc(i)},Pn.findDOMNode=function(n){if(n==null)return null;if(n.nodeType===1)return n;var i=n._reactInternals;if(i===void 0)throw typeof n.render=="function"?Error(t(188)):(n=Object.keys(n).join(","),Error(t(268,n)));return n=ie(i),n=n===null?null:n.stateNode,n},Pn.flushSync=function(n){return Ur(n)},Pn.hydrate=function(n,i,a){if(!al(i))throw Error(t(200));return ol(null,n,i,!0,a)},Pn.hydrateRoot=function(n,i,a){if(!Bc(n))throw Error(t(405));var l=a!=null&&a.hydratedSources||null,f=!1,p="",E=Jp;if(a!=null&&(a.unstable_strictMode===!0&&(f=!0),a.identifierPrefix!==void 0&&(p=a.identifierPrefix),a.onRecoverableError!==void 0&&(E=a.onRecoverableError)),i=Zp(i,null,n,1,a??null,f,!1,p,E),n[wi]=i.current,Ea(n),l)for(n=0;n<l.length;n++)a=l[n],f=a._getVersion,f=f(a._source),i.mutableSourceEagerHydrationData==null?i.mutableSourceEagerHydrationData=[a,f]:i.mutableSourceEagerHydrationData.push(a,f);return new sl(i)},Pn.render=function(n,i,a){if(!al(i))throw Error(t(200));return ol(null,n,i,!1,a)},Pn.unmountComponentAtNode=function(n){if(!al(n))throw Error(t(40));return n._reactRootContainer?(Ur(function(){ol(null,null,n,!1,function(){n._reactRootContainer=null,n[wi]=null})}),!0):!1},Pn.unstable_batchedUpdates=Pc,Pn.unstable_renderSubtreeIntoContainer=function(n,i,a,l){if(!al(a))throw Error(t(200));if(n==null||n._reactInternals===void 0)throw Error(t(38));return ol(n,i,a,!1,l)},Pn.version="18.3.1-next-f1338f8080-20240426",Pn}var lm;function Qv(){if(lm)return Vc.exports;lm=1;function s(){if(!(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__>"u"||typeof __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE!="function"))try{__REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE(s)}catch(e){console.error(e)}}return s(),Vc.exports=Zv(),Vc.exports}var um;function Jv(){if(um)return ul;um=1;var s=Qv();return ul.createRoot=s.createRoot,ul.hydrateRoot=s.hydrateRoot,ul}var e0=Jv();class cm{constructor(){this.ctx=null,this.armed=!0,this.quietAdvisories=!1,this._mode="off",this._osc=null,this._gain=null,this._lfo=null,this._timer=null,this._lastPubBeep=0,this._status={leak:!1,publisher:"never",stream:"connecting"},this._raf=null}async arm(){if(!this.ctx){const e=window.AudioContext||window.webkitAudioContext;this.ctx=new e}this.ctx.state==="suspended"&&await this.ctx.resume(),this.armed=!0,this._ensureLoop()}setQuietAdvisories(e){this.quietAdvisories=e,localStorage.setItem("rov.alert.quietAdvisories",e?"1":"0"),localStorage.removeItem("rov.alert.muted"),this._apply()}setMuted(e){this.setQuietAdvisories(e)}get muted(){return this.quietAdvisories}set muted(e){this.quietAdvisories=!!e}static loadPrefs(){const e=localStorage.getItem("rov.alert.muted")==="1",t=localStorage.getItem("rov.alert.quietAdvisories")==="1"||e;return{muted:t,quietAdvisories:t}}update(e){this._status={...this._status,...e},this._ensureLoop(),this._apply()}_ensureLoop(){if(this._raf!=null)return;const e=()=>{this._raf=window.setTimeout(e,500),this._apply()};this._raf=window.setTimeout(e,500)}_apply(){if(!this.armed||!this.ctx){this._stop();return}this.ctx.state==="suspended"&&this.ctx.resume().catch(()=>{});const{leak:e,publisher:t,stream:r}=this._status;if(e){this._siren();return}if(this.quietAdvisories){this._stop();return}if(r==="disconnected"){this._pulse(180,5);return}if(t==="stale"||t==="never"){const o=performance.now()/1e3;o-this._lastPubBeep>20&&(this._lastPubBeep=o,this._beep(420,.2)),(this._mode==="siren"||this._mode==="pulse")&&this._stop();return}this._stop()}_ensureVoice(){if(this._osc)return;const e=this.ctx;this._osc=e.createOscillator(),this._gain=e.createGain(),this._osc.type="square",this._gain.gain.value=0,this._osc.connect(this._gain),this._gain.connect(e.destination),this._osc.start()}_siren(){if(this._ensureVoice(),this._mode==="siren")return;this._stopLfo(),this._mode="siren",this._osc.type="square";const e=this.ctx.currentTime;this._gain.gain.cancelScheduledValues(e),this._gain.gain.setValueAtTime(0,e),this._gain.gain.linearRampToValueAtTime(.12,e+.05),this._lfo=this.ctx.createOscillator();const t=this.ctx.createGain();this._lfo.frequency.value=4,t.gain.value=180,this._osc.frequency.value=620,this._lfo.connect(t),t.connect(this._osc.frequency),this._lfo.start(),this._lfoGain=t}_pulse(e,t){if(this._ensureVoice(),this._mode==="pulse")return;this._stopLfo(),this._mode="pulse",this._osc.type="sawtooth",this._osc.frequency.value=e;const r=()=>{if(this._mode!=="pulse"||!this._gain)return;const o=this.ctx.currentTime;this._gain.gain.cancelScheduledValues(o),this._gain.gain.setValueAtTime(0,o),this._gain.gain.linearRampToValueAtTime(.07,o+.02),this._gain.gain.linearRampToValueAtTime(0,o+.18)};r(),this._timer=window.setInterval(r,t*1e3)}_beep(e,t){if(!this.ctx)return;const r=this.ctx.createOscillator(),o=this.ctx.createGain();r.frequency.value=e,r.type="triangle",o.gain.value=.06,r.connect(o),o.connect(this.ctx.destination),r.start(),o.gain.exponentialRampToValueAtTime(.001,this.ctx.currentTime+t),r.stop(this.ctx.currentTime+t)}_stopLfo(){if(this._lfo){try{this._lfo.stop()}catch{}this._lfo.disconnect(),this._lfo=null}this._lfoGain&&(this._lfoGain.disconnect(),this._lfoGain=null),this._timer&&(clearInterval(this._timer),this._timer=null)}_stop(){this._stopLfo(),this._gain&&this._gain.gain.setTargetAtTime(0,this.ctx?this.ctx.currentTime:0,.02),this._mode="off"}}/**
 * @license
 * Copyright 2010-2024 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const md="167",t0=0,fm=1,n0=2,wg=1,i0=2,Oi=3,Mr=0,Ln=1,ki=2,xr=0,Zs=1,dm=2,hm=3,pm=4,r0=5,Yr=100,s0=101,a0=102,o0=103,l0=104,u0=200,c0=201,f0=202,d0=203,Lf=204,Nf=205,h0=206,p0=207,m0=208,g0=209,_0=210,v0=211,x0=212,y0=213,S0=214,M0=0,E0=1,w0=2,jl=3,T0=4,A0=5,C0=6,R0=7,Tg=0,b0=1,P0=2,yr=0,L0=1,N0=2,D0=3,I0=4,U0=5,F0=6,O0=7,Ag=300,ta=301,na=302,Df=303,If=304,tu=306,Uf=1e3,$r=1001,Ff=1002,Kn=1003,k0=1004,cl=1005,ui=1006,Xc=1007,Kr=1008,Hi=1009,Cg=1010,Rg=1011,Qa=1012,gd=1013,Jr=1014,Bi=1015,eo=1016,_d=1017,vd=1018,ia=1020,bg=35902,Pg=1021,Lg=1022,fi=1023,Ng=1024,Dg=1025,Qs=1026,ra=1027,Ig=1028,xd=1029,Ug=1030,yd=1031,Sd=1033,zl=33776,Hl=33777,Vl=33778,Gl=33779,Of=35840,kf=35841,Bf=35842,zf=35843,Hf=36196,Vf=37492,Gf=37496,Wf=37808,Xf=37809,jf=37810,Yf=37811,qf=37812,$f=37813,Kf=37814,Zf=37815,Qf=37816,Jf=37817,ed=37818,td=37819,nd=37820,id=37821,Wl=36492,rd=36494,sd=36495,Fg=36283,ad=36284,od=36285,ld=36286,B0=3200,z0=3201,Og=0,H0=1,vr="",li="srgb",wr="srgb-linear",Md="display-p3",nu="display-p3-linear",Yl="linear",Ot="srgb",ql="rec709",$l="p3",As=7680,mm=519,V0=512,G0=513,W0=514,kg=515,X0=516,j0=517,Y0=518,q0=519,ud=35044,gm="300 es",zi=2e3,Kl=2001;class aa{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});const r=this._listeners;r[e]===void 0&&(r[e]=[]),r[e].indexOf(t)===-1&&r[e].push(t)}hasEventListener(e,t){if(this._listeners===void 0)return!1;const r=this._listeners;return r[e]!==void 0&&r[e].indexOf(t)!==-1}removeEventListener(e,t){if(this._listeners===void 0)return;const o=this._listeners[e];if(o!==void 0){const u=o.indexOf(t);u!==-1&&o.splice(u,1)}}dispatchEvent(e){if(this._listeners===void 0)return;const r=this._listeners[e.type];if(r!==void 0){e.target=this;const o=r.slice(0);for(let u=0,c=o.length;u<c;u++)o[u].call(this,e);e.target=null}}}const pn=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],jc=Math.PI/180,cd=180/Math.PI;function Sr(){const s=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,r=Math.random()*4294967295|0;return(pn[s&255]+pn[s>>8&255]+pn[s>>16&255]+pn[s>>24&255]+"-"+pn[e&255]+pn[e>>8&255]+"-"+pn[e>>16&15|64]+pn[e>>24&255]+"-"+pn[t&63|128]+pn[t>>8&255]+"-"+pn[t>>16&255]+pn[t>>24&255]+pn[r&255]+pn[r>>8&255]+pn[r>>16&255]+pn[r>>24&255]).toLowerCase()}function Mn(s,e,t){return Math.max(e,Math.min(t,s))}function $0(s,e){return(s%e+e)%e}function Yc(s,e,t){return(1-t)*s+t*e}function xi(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return s/4294967295;case Uint16Array:return s/65535;case Uint8Array:return s/255;case Int32Array:return Math.max(s/2147483647,-1);case Int16Array:return Math.max(s/32767,-1);case Int8Array:return Math.max(s/127,-1);default:throw new Error("Invalid component type.")}}function Pt(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return Math.round(s*4294967295);case Uint16Array:return Math.round(s*65535);case Uint8Array:return Math.round(s*255);case Int32Array:return Math.round(s*2147483647);case Int16Array:return Math.round(s*32767);case Int8Array:return Math.round(s*127);default:throw new Error("Invalid component type.")}}class ft{constructor(e=0,t=0){ft.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){const t=this.x,r=this.y,o=e.elements;return this.x=o[0]*t+o[3]*r+o[6],this.y=o[1]*t+o[4]*r+o[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const r=this.dot(e)/t;return Math.acos(Mn(r,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,r=this.y-e.y;return t*t+r*r}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){const r=Math.cos(t),o=Math.sin(t),u=this.x-e.x,c=this.y-e.y;return this.x=u*r-c*o+e.x,this.y=u*o+c*r+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ht{constructor(e,t,r,o,u,c,d,h,m){ht.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,r,o,u,c,d,h,m)}set(e,t,r,o,u,c,d,h,m){const g=this.elements;return g[0]=e,g[1]=o,g[2]=d,g[3]=t,g[4]=u,g[5]=h,g[6]=r,g[7]=c,g[8]=m,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){const t=this.elements,r=e.elements;return t[0]=r[0],t[1]=r[1],t[2]=r[2],t[3]=r[3],t[4]=r[4],t[5]=r[5],t[6]=r[6],t[7]=r[7],t[8]=r[8],this}extractBasis(e,t,r){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),r.setFromMatrix3Column(this,2),this}setFromMatrix4(e){const t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const r=e.elements,o=t.elements,u=this.elements,c=r[0],d=r[3],h=r[6],m=r[1],g=r[4],y=r[7],v=r[2],M=r[5],T=r[8],S=o[0],x=o[3],_=o[6],P=o[1],R=o[4],L=o[7],$=o[2],O=o[5],D=o[8];return u[0]=c*S+d*P+h*$,u[3]=c*x+d*R+h*O,u[6]=c*_+d*L+h*D,u[1]=m*S+g*P+y*$,u[4]=m*x+g*R+y*O,u[7]=m*_+g*L+y*D,u[2]=v*S+M*P+T*$,u[5]=v*x+M*R+T*O,u[8]=v*_+M*L+T*D,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],d=e[5],h=e[6],m=e[7],g=e[8];return t*c*g-t*d*m-r*u*g+r*d*h+o*u*m-o*c*h}invert(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],d=e[5],h=e[6],m=e[7],g=e[8],y=g*c-d*m,v=d*h-g*u,M=m*u-c*h,T=t*y+r*v+o*M;if(T===0)return this.set(0,0,0,0,0,0,0,0,0);const S=1/T;return e[0]=y*S,e[1]=(o*m-g*r)*S,e[2]=(d*r-o*c)*S,e[3]=v*S,e[4]=(g*t-o*h)*S,e[5]=(o*u-d*t)*S,e[6]=M*S,e[7]=(r*h-m*t)*S,e[8]=(c*t-r*u)*S,this}transpose(){let e;const t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){const t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,r,o,u,c,d){const h=Math.cos(u),m=Math.sin(u);return this.set(r*h,r*m,-r*(h*c+m*d)+c+e,-o*m,o*h,-o*(-m*c+h*d)+d+t,0,0,1),this}scale(e,t){return this.premultiply(qc.makeScale(e,t)),this}rotate(e){return this.premultiply(qc.makeRotation(-e)),this}translate(e,t){return this.premultiply(qc.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,-r,0,r,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){const t=this.elements,r=e.elements;for(let o=0;o<9;o++)if(t[o]!==r[o])return!1;return!0}fromArray(e,t=0){for(let r=0;r<9;r++)this.elements[r]=e[r+t];return this}toArray(e=[],t=0){const r=this.elements;return e[t]=r[0],e[t+1]=r[1],e[t+2]=r[2],e[t+3]=r[3],e[t+4]=r[4],e[t+5]=r[5],e[t+6]=r[6],e[t+7]=r[7],e[t+8]=r[8],e}clone(){return new this.constructor().fromArray(this.elements)}}const qc=new ht;function Bg(s){for(let e=s.length-1;e>=0;--e)if(s[e]>=65535)return!0;return!1}function Zl(s){return document.createElementNS("http://www.w3.org/1999/xhtml",s)}function K0(){const s=Zl("canvas");return s.style.display="block",s}const _m={};function Js(s){s in _m||(_m[s]=!0,console.warn(s))}function Z0(s,e,t){return new Promise(function(r,o){function u(){switch(s.clientWaitSync(e,s.SYNC_FLUSH_COMMANDS_BIT,0)){case s.WAIT_FAILED:o();break;case s.TIMEOUT_EXPIRED:setTimeout(u,t);break;default:r()}}setTimeout(u,t)})}const vm=new ht().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),xm=new ht().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Va={[wr]:{transfer:Yl,primaries:ql,luminanceCoefficients:[.2126,.7152,.0722],toReference:s=>s,fromReference:s=>s},[li]:{transfer:Ot,primaries:ql,luminanceCoefficients:[.2126,.7152,.0722],toReference:s=>s.convertSRGBToLinear(),fromReference:s=>s.convertLinearToSRGB()},[nu]:{transfer:Yl,primaries:$l,luminanceCoefficients:[.2289,.6917,.0793],toReference:s=>s.applyMatrix3(xm),fromReference:s=>s.applyMatrix3(vm)},[Md]:{transfer:Ot,primaries:$l,luminanceCoefficients:[.2289,.6917,.0793],toReference:s=>s.convertSRGBToLinear().applyMatrix3(xm),fromReference:s=>s.applyMatrix3(vm).convertLinearToSRGB()}},Q0=new Set([wr,nu]),At={enabled:!0,_workingColorSpace:wr,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(s){if(!Q0.has(s))throw new Error(`Unsupported working color space, "${s}".`);this._workingColorSpace=s},convert:function(s,e,t){if(this.enabled===!1||e===t||!e||!t)return s;const r=Va[e].toReference,o=Va[t].fromReference;return o(r(s))},fromWorkingColorSpace:function(s,e){return this.convert(s,this._workingColorSpace,e)},toWorkingColorSpace:function(s,e){return this.convert(s,e,this._workingColorSpace)},getPrimaries:function(s){return Va[s].primaries},getTransfer:function(s){return s===vr?Yl:Va[s].transfer},getLuminanceCoefficients:function(s,e=this._workingColorSpace){return s.fromArray(Va[e].luminanceCoefficients)}};function ea(s){return s<.04045?s*.0773993808:Math.pow(s*.9478672986+.0521327014,2.4)}function $c(s){return s<.0031308?s*12.92:1.055*Math.pow(s,.41666)-.055}let Cs;class J0{static getDataURL(e){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let t;if(e instanceof HTMLCanvasElement)t=e;else{Cs===void 0&&(Cs=Zl("canvas")),Cs.width=e.width,Cs.height=e.height;const r=Cs.getContext("2d");e instanceof ImageData?r.putImageData(e,0,0):r.drawImage(e,0,0,e.width,e.height),t=Cs}return t.width>2048||t.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",e),t.toDataURL("image/jpeg",.6)):t.toDataURL("image/png")}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){const t=Zl("canvas");t.width=e.width,t.height=e.height;const r=t.getContext("2d");r.drawImage(e,0,0,e.width,e.height);const o=r.getImageData(0,0,e.width,e.height),u=o.data;for(let c=0;c<u.length;c++)u[c]=ea(u[c]/255)*255;return r.putImageData(o,0,0),t}else if(e.data){const t=e.data.slice(0);for(let r=0;r<t.length;r++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[r]=Math.floor(ea(t[r]/255)*255):t[r]=ea(t[r]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}}let ex=0;class zg{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:ex++}),this.uuid=Sr(),this.data=e,this.dataReady=!0,this.version=0}set needsUpdate(e){e===!0&&this.version++}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];const r={uuid:this.uuid,url:""},o=this.data;if(o!==null){let u;if(Array.isArray(o)){u=[];for(let c=0,d=o.length;c<d;c++)o[c].isDataTexture?u.push(Kc(o[c].image)):u.push(Kc(o[c]))}else u=Kc(o);r.url=u}return t||(e.images[this.uuid]=r),r}}function Kc(s){return typeof HTMLImageElement<"u"&&s instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&s instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&s instanceof ImageBitmap?J0.getDataURL(s):s.data?{data:Array.from(s.data),width:s.width,height:s.height,type:s.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let tx=0;class En extends aa{constructor(e=En.DEFAULT_IMAGE,t=En.DEFAULT_MAPPING,r=$r,o=$r,u=ui,c=Kr,d=fi,h=Hi,m=En.DEFAULT_ANISOTROPY,g=vr){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:tx++}),this.uuid=Sr(),this.name="",this.source=new zg(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=r,this.wrapT=o,this.magFilter=u,this.minFilter=c,this.anisotropy=m,this.format=d,this.internalFormat=null,this.type=h,this.offset=new ft(0,0),this.repeat=new ft(1,1),this.center=new ft(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ht,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=g,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];const r={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(r.userData=this.userData),t||(e.textures[this.uuid]=r),r}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==Ag)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case Uf:e.x=e.x-Math.floor(e.x);break;case $r:e.x=e.x<0?0:1;break;case Ff:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case Uf:e.y=e.y-Math.floor(e.y);break;case $r:e.y=e.y<0?0:1;break;case Ff:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}}En.DEFAULT_IMAGE=null;En.DEFAULT_MAPPING=Ag;En.DEFAULT_ANISOTROPY=1;class Zt{constructor(e=0,t=0,r=0,o=1){Zt.prototype.isVector4=!0,this.x=e,this.y=t,this.z=r,this.w=o}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,r,o){return this.x=e,this.y=t,this.z=r,this.w=o,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){const t=this.x,r=this.y,o=this.z,u=this.w,c=e.elements;return this.x=c[0]*t+c[4]*r+c[8]*o+c[12]*u,this.y=c[1]*t+c[5]*r+c[9]*o+c[13]*u,this.z=c[2]*t+c[6]*r+c[10]*o+c[14]*u,this.w=c[3]*t+c[7]*r+c[11]*o+c[15]*u,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);const t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,r,o,u;const h=e.elements,m=h[0],g=h[4],y=h[8],v=h[1],M=h[5],T=h[9],S=h[2],x=h[6],_=h[10];if(Math.abs(g-v)<.01&&Math.abs(y-S)<.01&&Math.abs(T-x)<.01){if(Math.abs(g+v)<.1&&Math.abs(y+S)<.1&&Math.abs(T+x)<.1&&Math.abs(m+M+_-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;const R=(m+1)/2,L=(M+1)/2,$=(_+1)/2,O=(g+v)/4,D=(y+S)/4,j=(T+x)/4;return R>L&&R>$?R<.01?(r=0,o=.707106781,u=.707106781):(r=Math.sqrt(R),o=O/r,u=D/r):L>$?L<.01?(r=.707106781,o=0,u=.707106781):(o=Math.sqrt(L),r=O/o,u=j/o):$<.01?(r=.707106781,o=.707106781,u=0):(u=Math.sqrt($),r=D/u,o=j/u),this.set(r,o,u,t),this}let P=Math.sqrt((x-T)*(x-T)+(y-S)*(y-S)+(v-g)*(v-g));return Math.abs(P)<.001&&(P=1),this.x=(x-T)/P,this.y=(y-S)/P,this.z=(v-g)/P,this.w=Math.acos((m+M+_-1)/2),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this.w=t[15],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this.w=Math.max(e.w,Math.min(t.w,this.w)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this.w=Math.max(e,Math.min(t,this.w)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this.z=e.z+(t.z-e.z)*r,this.w=e.w+(t.w-e.w)*r,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class nx extends aa{constructor(e=1,t=1,r={}){super(),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=1,this.scissor=new Zt(0,0,e,t),this.scissorTest=!1,this.viewport=new Zt(0,0,e,t);const o={width:e,height:t,depth:1};r=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:ui,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},r);const u=new En(o,r.mapping,r.wrapS,r.wrapT,r.magFilter,r.minFilter,r.format,r.type,r.anisotropy,r.colorSpace);u.flipY=!1,u.generateMipmaps=r.generateMipmaps,u.internalFormat=r.internalFormat,this.textures=[];const c=r.count;for(let d=0;d<c;d++)this.textures[d]=u.clone(),this.textures[d].isRenderTargetTexture=!0;this.depthBuffer=r.depthBuffer,this.stencilBuffer=r.stencilBuffer,this.resolveDepthBuffer=r.resolveDepthBuffer,this.resolveStencilBuffer=r.resolveStencilBuffer,this.depthTexture=r.depthTexture,this.samples=r.samples}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}setSize(e,t,r=1){if(this.width!==e||this.height!==t||this.depth!==r){this.width=e,this.height=t,this.depth=r;for(let o=0,u=this.textures.length;o<u;o++)this.textures[o].image.width=e,this.textures[o].image.height=t,this.textures[o].image.depth=r;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let r=0,o=e.textures.length;r<o;r++)this.textures[r]=e.textures[r].clone(),this.textures[r].isRenderTargetTexture=!0;const t=Object.assign({},e.texture.image);return this.texture.source=new zg(t),this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class es extends nx{constructor(e=1,t=1,r={}){super(e,t,r),this.isWebGLRenderTarget=!0}}class Hg extends En{constructor(e=null,t=1,r=1,o=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:r,depth:o},this.magFilter=Kn,this.minFilter=Kn,this.wrapR=$r,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}}class ix extends En{constructor(e=null,t=1,r=1,o=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:r,depth:o},this.magFilter=Kn,this.minFilter=Kn,this.wrapR=$r,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class to{constructor(e=0,t=0,r=0,o=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=r,this._w=o}static slerpFlat(e,t,r,o,u,c,d){let h=r[o+0],m=r[o+1],g=r[o+2],y=r[o+3];const v=u[c+0],M=u[c+1],T=u[c+2],S=u[c+3];if(d===0){e[t+0]=h,e[t+1]=m,e[t+2]=g,e[t+3]=y;return}if(d===1){e[t+0]=v,e[t+1]=M,e[t+2]=T,e[t+3]=S;return}if(y!==S||h!==v||m!==M||g!==T){let x=1-d;const _=h*v+m*M+g*T+y*S,P=_>=0?1:-1,R=1-_*_;if(R>Number.EPSILON){const $=Math.sqrt(R),O=Math.atan2($,_*P);x=Math.sin(x*O)/$,d=Math.sin(d*O)/$}const L=d*P;if(h=h*x+v*L,m=m*x+M*L,g=g*x+T*L,y=y*x+S*L,x===1-d){const $=1/Math.sqrt(h*h+m*m+g*g+y*y);h*=$,m*=$,g*=$,y*=$}}e[t]=h,e[t+1]=m,e[t+2]=g,e[t+3]=y}static multiplyQuaternionsFlat(e,t,r,o,u,c){const d=r[o],h=r[o+1],m=r[o+2],g=r[o+3],y=u[c],v=u[c+1],M=u[c+2],T=u[c+3];return e[t]=d*T+g*y+h*M-m*v,e[t+1]=h*T+g*v+m*y-d*M,e[t+2]=m*T+g*M+d*v-h*y,e[t+3]=g*T-d*y-h*v-m*M,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,r,o){return this._x=e,this._y=t,this._z=r,this._w=o,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){const r=e._x,o=e._y,u=e._z,c=e._order,d=Math.cos,h=Math.sin,m=d(r/2),g=d(o/2),y=d(u/2),v=h(r/2),M=h(o/2),T=h(u/2);switch(c){case"XYZ":this._x=v*g*y+m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y-v*M*T;break;case"YXZ":this._x=v*g*y+m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y+v*M*T;break;case"ZXY":this._x=v*g*y-m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y-v*M*T;break;case"ZYX":this._x=v*g*y-m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y+v*M*T;break;case"YZX":this._x=v*g*y+m*M*T,this._y=m*M*y+v*g*T,this._z=m*g*T-v*M*y,this._w=m*g*y-v*M*T;break;case"XZY":this._x=v*g*y-m*M*T,this._y=m*M*y-v*g*T,this._z=m*g*T+v*M*y,this._w=m*g*y+v*M*T;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+c)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){const r=t/2,o=Math.sin(r);return this._x=e.x*o,this._y=e.y*o,this._z=e.z*o,this._w=Math.cos(r),this._onChangeCallback(),this}setFromRotationMatrix(e){const t=e.elements,r=t[0],o=t[4],u=t[8],c=t[1],d=t[5],h=t[9],m=t[2],g=t[6],y=t[10],v=r+d+y;if(v>0){const M=.5/Math.sqrt(v+1);this._w=.25/M,this._x=(g-h)*M,this._y=(u-m)*M,this._z=(c-o)*M}else if(r>d&&r>y){const M=2*Math.sqrt(1+r-d-y);this._w=(g-h)/M,this._x=.25*M,this._y=(o+c)/M,this._z=(u+m)/M}else if(d>y){const M=2*Math.sqrt(1+d-r-y);this._w=(u-m)/M,this._x=(o+c)/M,this._y=.25*M,this._z=(h+g)/M}else{const M=2*Math.sqrt(1+y-r-d);this._w=(c-o)/M,this._x=(u+m)/M,this._y=(h+g)/M,this._z=.25*M}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let r=e.dot(t)+1;return r<Number.EPSILON?(r=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=r):(this._x=0,this._y=-e.z,this._z=e.y,this._w=r)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=r),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(Mn(this.dot(e),-1,1)))}rotateTowards(e,t){const r=this.angleTo(e);if(r===0)return this;const o=Math.min(1,t/r);return this.slerp(e,o),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){const r=e._x,o=e._y,u=e._z,c=e._w,d=t._x,h=t._y,m=t._z,g=t._w;return this._x=r*g+c*d+o*m-u*h,this._y=o*g+c*h+u*d-r*m,this._z=u*g+c*m+r*h-o*d,this._w=c*g-r*d-o*h-u*m,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);const r=this._x,o=this._y,u=this._z,c=this._w;let d=c*e._w+r*e._x+o*e._y+u*e._z;if(d<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,d=-d):this.copy(e),d>=1)return this._w=c,this._x=r,this._y=o,this._z=u,this;const h=1-d*d;if(h<=Number.EPSILON){const M=1-t;return this._w=M*c+t*this._w,this._x=M*r+t*this._x,this._y=M*o+t*this._y,this._z=M*u+t*this._z,this.normalize(),this}const m=Math.sqrt(h),g=Math.atan2(m,d),y=Math.sin((1-t)*g)/m,v=Math.sin(t*g)/m;return this._w=c*y+this._w*v,this._x=r*y+this._x*v,this._y=o*y+this._y*v,this._z=u*y+this._z*v,this._onChangeCallback(),this}slerpQuaternions(e,t,r){return this.copy(e).slerp(t,r)}random(){const e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),r=Math.random(),o=Math.sqrt(1-r),u=Math.sqrt(r);return this.set(o*Math.sin(e),o*Math.cos(e),u*Math.sin(t),u*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class J{constructor(e=0,t=0,r=0){J.prototype.isVector3=!0,this.x=e,this.y=t,this.z=r}set(e,t,r){return r===void 0&&(r=this.z),this.x=e,this.y=t,this.z=r,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(ym.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(ym.setFromAxisAngle(e,t))}applyMatrix3(e){const t=this.x,r=this.y,o=this.z,u=e.elements;return this.x=u[0]*t+u[3]*r+u[6]*o,this.y=u[1]*t+u[4]*r+u[7]*o,this.z=u[2]*t+u[5]*r+u[8]*o,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){const t=this.x,r=this.y,o=this.z,u=e.elements,c=1/(u[3]*t+u[7]*r+u[11]*o+u[15]);return this.x=(u[0]*t+u[4]*r+u[8]*o+u[12])*c,this.y=(u[1]*t+u[5]*r+u[9]*o+u[13])*c,this.z=(u[2]*t+u[6]*r+u[10]*o+u[14])*c,this}applyQuaternion(e){const t=this.x,r=this.y,o=this.z,u=e.x,c=e.y,d=e.z,h=e.w,m=2*(c*o-d*r),g=2*(d*t-u*o),y=2*(u*r-c*t);return this.x=t+h*m+c*y-d*g,this.y=r+h*g+d*m-u*y,this.z=o+h*y+u*g-c*m,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){const t=this.x,r=this.y,o=this.z,u=e.elements;return this.x=u[0]*t+u[4]*r+u[8]*o,this.y=u[1]*t+u[5]*r+u[9]*o,this.z=u[2]*t+u[6]*r+u[10]*o,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this}clampLength(e,t){const r=this.length();return this.divideScalar(r||1).multiplyScalar(Math.max(e,Math.min(t,r)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,r){return this.x=e.x+(t.x-e.x)*r,this.y=e.y+(t.y-e.y)*r,this.z=e.z+(t.z-e.z)*r,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){const r=e.x,o=e.y,u=e.z,c=t.x,d=t.y,h=t.z;return this.x=o*h-u*d,this.y=u*c-r*h,this.z=r*d-o*c,this}projectOnVector(e){const t=e.lengthSq();if(t===0)return this.set(0,0,0);const r=e.dot(this)/t;return this.copy(e).multiplyScalar(r)}projectOnPlane(e){return Zc.copy(this).projectOnVector(e),this.sub(Zc)}reflect(e){return this.sub(Zc.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const r=this.dot(e)/t;return Math.acos(Mn(r,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,r=this.y-e.y,o=this.z-e.z;return t*t+r*r+o*o}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,r){const o=Math.sin(t)*e;return this.x=o*Math.sin(r),this.y=Math.cos(t)*e,this.z=o*Math.cos(r),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,r){return this.x=e*Math.sin(t),this.y=r,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){const t=this.setFromMatrixColumn(e,0).length(),r=this.setFromMatrixColumn(e,1).length(),o=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=r,this.z=o,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const e=Math.random()*Math.PI*2,t=Math.random()*2-1,r=Math.sqrt(1-t*t);return this.x=r*Math.cos(e),this.y=t,this.z=r*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const Zc=new J,ym=new to;class no{constructor(e=new J(1/0,1/0,1/0),t=new J(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,r=e.length;t<r;t+=3)this.expandByPoint(si.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,r=e.count;t<r;t++)this.expandByPoint(si.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,r=e.length;t<r;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){const r=si.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(r),this.max.copy(e).add(r),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);const r=e.geometry;if(r!==void 0){const u=r.getAttribute("position");if(t===!0&&u!==void 0&&e.isInstancedMesh!==!0)for(let c=0,d=u.count;c<d;c++)e.isMesh===!0?e.getVertexPosition(c,si):si.fromBufferAttribute(u,c),si.applyMatrix4(e.matrixWorld),this.expandByPoint(si);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),fl.copy(e.boundingBox)):(r.boundingBox===null&&r.computeBoundingBox(),fl.copy(r.boundingBox)),fl.applyMatrix4(e.matrixWorld),this.union(fl)}const o=e.children;for(let u=0,c=o.length;u<c;u++)this.expandByObject(o[u],t);return this}containsPoint(e){return e.x>=this.min.x&&e.x<=this.max.x&&e.y>=this.min.y&&e.y<=this.max.y&&e.z>=this.min.z&&e.z<=this.max.z}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return e.max.x>=this.min.x&&e.min.x<=this.max.x&&e.max.y>=this.min.y&&e.min.y<=this.max.y&&e.max.z>=this.min.z&&e.min.z<=this.max.z}intersectsSphere(e){return this.clampPoint(e.center,si),si.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,r;return e.normal.x>0?(t=e.normal.x*this.min.x,r=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,r=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,r+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,r+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,r+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,r+=e.normal.z*this.min.z),t<=-e.constant&&r>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(Ga),dl.subVectors(this.max,Ga),Rs.subVectors(e.a,Ga),bs.subVectors(e.b,Ga),Ps.subVectors(e.c,Ga),dr.subVectors(bs,Rs),hr.subVectors(Ps,bs),Br.subVectors(Rs,Ps);let t=[0,-dr.z,dr.y,0,-hr.z,hr.y,0,-Br.z,Br.y,dr.z,0,-dr.x,hr.z,0,-hr.x,Br.z,0,-Br.x,-dr.y,dr.x,0,-hr.y,hr.x,0,-Br.y,Br.x,0];return!Qc(t,Rs,bs,Ps,dl)||(t=[1,0,0,0,1,0,0,0,1],!Qc(t,Rs,bs,Ps,dl))?!1:(hl.crossVectors(dr,hr),t=[hl.x,hl.y,hl.z],Qc(t,Rs,bs,Ps,dl))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,si).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(si).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(Ni[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),Ni[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),Ni[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),Ni[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),Ni[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),Ni[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),Ni[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),Ni[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(Ni),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}}const Ni=[new J,new J,new J,new J,new J,new J,new J,new J],si=new J,fl=new no,Rs=new J,bs=new J,Ps=new J,dr=new J,hr=new J,Br=new J,Ga=new J,dl=new J,hl=new J,zr=new J;function Qc(s,e,t,r,o){for(let u=0,c=s.length-3;u<=c;u+=3){zr.fromArray(s,u);const d=o.x*Math.abs(zr.x)+o.y*Math.abs(zr.y)+o.z*Math.abs(zr.z),h=e.dot(zr),m=t.dot(zr),g=r.dot(zr);if(Math.max(-Math.max(h,m,g),Math.min(h,m,g))>d)return!1}return!0}const rx=new no,Wa=new J,Jc=new J;class iu{constructor(e=new J,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){const r=this.center;t!==void 0?r.copy(t):rx.setFromPoints(e).getCenter(r);let o=0;for(let u=0,c=e.length;u<c;u++)o=Math.max(o,r.distanceToSquared(e[u]));return this.radius=Math.sqrt(o),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){const t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){const r=this.center.distanceToSquared(e);return t.copy(e),r>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;Wa.subVectors(e,this.center);const t=Wa.lengthSq();if(t>this.radius*this.radius){const r=Math.sqrt(t),o=(r-this.radius)*.5;this.center.addScaledVector(Wa,o/r),this.radius+=o}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(Jc.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(Wa.copy(e.center).add(Jc)),this.expandByPoint(Wa.copy(e.center).sub(Jc))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}}const Di=new J,ef=new J,pl=new J,pr=new J,tf=new J,ml=new J,nf=new J;class Vg{constructor(e=new J,t=new J(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,Di)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);const r=t.dot(this.direction);return r<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,r)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){const t=Di.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(Di.copy(this.origin).addScaledVector(this.direction,t),Di.distanceToSquared(e))}distanceSqToSegment(e,t,r,o){ef.copy(e).add(t).multiplyScalar(.5),pl.copy(t).sub(e).normalize(),pr.copy(this.origin).sub(ef);const u=e.distanceTo(t)*.5,c=-this.direction.dot(pl),d=pr.dot(this.direction),h=-pr.dot(pl),m=pr.lengthSq(),g=Math.abs(1-c*c);let y,v,M,T;if(g>0)if(y=c*h-d,v=c*d-h,T=u*g,y>=0)if(v>=-T)if(v<=T){const S=1/g;y*=S,v*=S,M=y*(y+c*v+2*d)+v*(c*y+v+2*h)+m}else v=u,y=Math.max(0,-(c*v+d)),M=-y*y+v*(v+2*h)+m;else v=-u,y=Math.max(0,-(c*v+d)),M=-y*y+v*(v+2*h)+m;else v<=-T?(y=Math.max(0,-(-c*u+d)),v=y>0?-u:Math.min(Math.max(-u,-h),u),M=-y*y+v*(v+2*h)+m):v<=T?(y=0,v=Math.min(Math.max(-u,-h),u),M=v*(v+2*h)+m):(y=Math.max(0,-(c*u+d)),v=y>0?u:Math.min(Math.max(-u,-h),u),M=-y*y+v*(v+2*h)+m);else v=c>0?-u:u,y=Math.max(0,-(c*v+d)),M=-y*y+v*(v+2*h)+m;return r&&r.copy(this.origin).addScaledVector(this.direction,y),o&&o.copy(ef).addScaledVector(pl,v),M}intersectSphere(e,t){Di.subVectors(e.center,this.origin);const r=Di.dot(this.direction),o=Di.dot(Di)-r*r,u=e.radius*e.radius;if(o>u)return null;const c=Math.sqrt(u-o),d=r-c,h=r+c;return h<0?null:d<0?this.at(h,t):this.at(d,t)}intersectsSphere(e){return this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){const t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;const r=-(this.origin.dot(e.normal)+e.constant)/t;return r>=0?r:null}intersectPlane(e,t){const r=this.distanceToPlane(e);return r===null?null:this.at(r,t)}intersectsPlane(e){const t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let r,o,u,c,d,h;const m=1/this.direction.x,g=1/this.direction.y,y=1/this.direction.z,v=this.origin;return m>=0?(r=(e.min.x-v.x)*m,o=(e.max.x-v.x)*m):(r=(e.max.x-v.x)*m,o=(e.min.x-v.x)*m),g>=0?(u=(e.min.y-v.y)*g,c=(e.max.y-v.y)*g):(u=(e.max.y-v.y)*g,c=(e.min.y-v.y)*g),r>c||u>o||((u>r||isNaN(r))&&(r=u),(c<o||isNaN(o))&&(o=c),y>=0?(d=(e.min.z-v.z)*y,h=(e.max.z-v.z)*y):(d=(e.max.z-v.z)*y,h=(e.min.z-v.z)*y),r>h||d>o)||((d>r||r!==r)&&(r=d),(h<o||o!==o)&&(o=h),o<0)?null:this.at(r>=0?r:o,t)}intersectsBox(e){return this.intersectBox(e,Di)!==null}intersectTriangle(e,t,r,o,u){tf.subVectors(t,e),ml.subVectors(r,e),nf.crossVectors(tf,ml);let c=this.direction.dot(nf),d;if(c>0){if(o)return null;d=1}else if(c<0)d=-1,c=-c;else return null;pr.subVectors(this.origin,e);const h=d*this.direction.dot(ml.crossVectors(pr,ml));if(h<0)return null;const m=d*this.direction.dot(tf.cross(pr));if(m<0||h+m>c)return null;const g=-d*pr.dot(nf);return g<0?null:this.at(g/c,u)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class Vt{constructor(e,t,r,o,u,c,d,h,m,g,y,v,M,T,S,x){Vt.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,r,o,u,c,d,h,m,g,y,v,M,T,S,x)}set(e,t,r,o,u,c,d,h,m,g,y,v,M,T,S,x){const _=this.elements;return _[0]=e,_[4]=t,_[8]=r,_[12]=o,_[1]=u,_[5]=c,_[9]=d,_[13]=h,_[2]=m,_[6]=g,_[10]=y,_[14]=v,_[3]=M,_[7]=T,_[11]=S,_[15]=x,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new Vt().fromArray(this.elements)}copy(e){const t=this.elements,r=e.elements;return t[0]=r[0],t[1]=r[1],t[2]=r[2],t[3]=r[3],t[4]=r[4],t[5]=r[5],t[6]=r[6],t[7]=r[7],t[8]=r[8],t[9]=r[9],t[10]=r[10],t[11]=r[11],t[12]=r[12],t[13]=r[13],t[14]=r[14],t[15]=r[15],this}copyPosition(e){const t=this.elements,r=e.elements;return t[12]=r[12],t[13]=r[13],t[14]=r[14],this}setFromMatrix3(e){const t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,r){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),r.setFromMatrixColumn(this,2),this}makeBasis(e,t,r){return this.set(e.x,t.x,r.x,0,e.y,t.y,r.y,0,e.z,t.z,r.z,0,0,0,0,1),this}extractRotation(e){const t=this.elements,r=e.elements,o=1/Ls.setFromMatrixColumn(e,0).length(),u=1/Ls.setFromMatrixColumn(e,1).length(),c=1/Ls.setFromMatrixColumn(e,2).length();return t[0]=r[0]*o,t[1]=r[1]*o,t[2]=r[2]*o,t[3]=0,t[4]=r[4]*u,t[5]=r[5]*u,t[6]=r[6]*u,t[7]=0,t[8]=r[8]*c,t[9]=r[9]*c,t[10]=r[10]*c,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){const t=this.elements,r=e.x,o=e.y,u=e.z,c=Math.cos(r),d=Math.sin(r),h=Math.cos(o),m=Math.sin(o),g=Math.cos(u),y=Math.sin(u);if(e.order==="XYZ"){const v=c*g,M=c*y,T=d*g,S=d*y;t[0]=h*g,t[4]=-h*y,t[8]=m,t[1]=M+T*m,t[5]=v-S*m,t[9]=-d*h,t[2]=S-v*m,t[6]=T+M*m,t[10]=c*h}else if(e.order==="YXZ"){const v=h*g,M=h*y,T=m*g,S=m*y;t[0]=v+S*d,t[4]=T*d-M,t[8]=c*m,t[1]=c*y,t[5]=c*g,t[9]=-d,t[2]=M*d-T,t[6]=S+v*d,t[10]=c*h}else if(e.order==="ZXY"){const v=h*g,M=h*y,T=m*g,S=m*y;t[0]=v-S*d,t[4]=-c*y,t[8]=T+M*d,t[1]=M+T*d,t[5]=c*g,t[9]=S-v*d,t[2]=-c*m,t[6]=d,t[10]=c*h}else if(e.order==="ZYX"){const v=c*g,M=c*y,T=d*g,S=d*y;t[0]=h*g,t[4]=T*m-M,t[8]=v*m+S,t[1]=h*y,t[5]=S*m+v,t[9]=M*m-T,t[2]=-m,t[6]=d*h,t[10]=c*h}else if(e.order==="YZX"){const v=c*h,M=c*m,T=d*h,S=d*m;t[0]=h*g,t[4]=S-v*y,t[8]=T*y+M,t[1]=y,t[5]=c*g,t[9]=-d*g,t[2]=-m*g,t[6]=M*y+T,t[10]=v-S*y}else if(e.order==="XZY"){const v=c*h,M=c*m,T=d*h,S=d*m;t[0]=h*g,t[4]=-y,t[8]=m*g,t[1]=v*y+S,t[5]=c*g,t[9]=M*y-T,t[2]=T*y-M,t[6]=d*g,t[10]=S*y+v}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(sx,e,ax)}lookAt(e,t,r){const o=this.elements;return kn.subVectors(e,t),kn.lengthSq()===0&&(kn.z=1),kn.normalize(),mr.crossVectors(r,kn),mr.lengthSq()===0&&(Math.abs(r.z)===1?kn.x+=1e-4:kn.z+=1e-4,kn.normalize(),mr.crossVectors(r,kn)),mr.normalize(),gl.crossVectors(kn,mr),o[0]=mr.x,o[4]=gl.x,o[8]=kn.x,o[1]=mr.y,o[5]=gl.y,o[9]=kn.y,o[2]=mr.z,o[6]=gl.z,o[10]=kn.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const r=e.elements,o=t.elements,u=this.elements,c=r[0],d=r[4],h=r[8],m=r[12],g=r[1],y=r[5],v=r[9],M=r[13],T=r[2],S=r[6],x=r[10],_=r[14],P=r[3],R=r[7],L=r[11],$=r[15],O=o[0],D=o[4],j=o[8],b=o[12],w=o[1],I=o[5],Y=o[9],K=o[13],oe=o[2],ne=o[6],B=o[10],G=o[14],k=o[3],ue=o[7],le=o[11],F=o[15];return u[0]=c*O+d*w+h*oe+m*k,u[4]=c*D+d*I+h*ne+m*ue,u[8]=c*j+d*Y+h*B+m*le,u[12]=c*b+d*K+h*G+m*F,u[1]=g*O+y*w+v*oe+M*k,u[5]=g*D+y*I+v*ne+M*ue,u[9]=g*j+y*Y+v*B+M*le,u[13]=g*b+y*K+v*G+M*F,u[2]=T*O+S*w+x*oe+_*k,u[6]=T*D+S*I+x*ne+_*ue,u[10]=T*j+S*Y+x*B+_*le,u[14]=T*b+S*K+x*G+_*F,u[3]=P*O+R*w+L*oe+$*k,u[7]=P*D+R*I+L*ne+$*ue,u[11]=P*j+R*Y+L*B+$*le,u[15]=P*b+R*K+L*G+$*F,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){const e=this.elements,t=e[0],r=e[4],o=e[8],u=e[12],c=e[1],d=e[5],h=e[9],m=e[13],g=e[2],y=e[6],v=e[10],M=e[14],T=e[3],S=e[7],x=e[11],_=e[15];return T*(+u*h*y-o*m*y-u*d*v+r*m*v+o*d*M-r*h*M)+S*(+t*h*M-t*m*v+u*c*v-o*c*M+o*m*g-u*h*g)+x*(+t*m*y-t*d*M-u*c*y+r*c*M+u*d*g-r*m*g)+_*(-o*d*g-t*h*y+t*d*v+o*c*y-r*c*v+r*h*g)}transpose(){const e=this.elements;let t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,r){const o=this.elements;return e.isVector3?(o[12]=e.x,o[13]=e.y,o[14]=e.z):(o[12]=e,o[13]=t,o[14]=r),this}invert(){const e=this.elements,t=e[0],r=e[1],o=e[2],u=e[3],c=e[4],d=e[5],h=e[6],m=e[7],g=e[8],y=e[9],v=e[10],M=e[11],T=e[12],S=e[13],x=e[14],_=e[15],P=y*x*m-S*v*m+S*h*M-d*x*M-y*h*_+d*v*_,R=T*v*m-g*x*m-T*h*M+c*x*M+g*h*_-c*v*_,L=g*S*m-T*y*m+T*d*M-c*S*M-g*d*_+c*y*_,$=T*y*h-g*S*h-T*d*v+c*S*v+g*d*x-c*y*x,O=t*P+r*R+o*L+u*$;if(O===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const D=1/O;return e[0]=P*D,e[1]=(S*v*u-y*x*u-S*o*M+r*x*M+y*o*_-r*v*_)*D,e[2]=(d*x*u-S*h*u+S*o*m-r*x*m-d*o*_+r*h*_)*D,e[3]=(y*h*u-d*v*u-y*o*m+r*v*m+d*o*M-r*h*M)*D,e[4]=R*D,e[5]=(g*x*u-T*v*u+T*o*M-t*x*M-g*o*_+t*v*_)*D,e[6]=(T*h*u-c*x*u-T*o*m+t*x*m+c*o*_-t*h*_)*D,e[7]=(c*v*u-g*h*u+g*o*m-t*v*m-c*o*M+t*h*M)*D,e[8]=L*D,e[9]=(T*y*u-g*S*u-T*r*M+t*S*M+g*r*_-t*y*_)*D,e[10]=(c*S*u-T*d*u+T*r*m-t*S*m-c*r*_+t*d*_)*D,e[11]=(g*d*u-c*y*u-g*r*m+t*y*m+c*r*M-t*d*M)*D,e[12]=$*D,e[13]=(g*S*o-T*y*o+T*r*v-t*S*v-g*r*x+t*y*x)*D,e[14]=(T*d*o-c*S*o-T*r*h+t*S*h+c*r*x-t*d*x)*D,e[15]=(c*y*o-g*d*o+g*r*h-t*y*h-c*r*v+t*d*v)*D,this}scale(e){const t=this.elements,r=e.x,o=e.y,u=e.z;return t[0]*=r,t[4]*=o,t[8]*=u,t[1]*=r,t[5]*=o,t[9]*=u,t[2]*=r,t[6]*=o,t[10]*=u,t[3]*=r,t[7]*=o,t[11]*=u,this}getMaxScaleOnAxis(){const e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],r=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],o=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,r,o))}makeTranslation(e,t,r){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,r,0,0,0,1),this}makeRotationX(e){const t=Math.cos(e),r=Math.sin(e);return this.set(1,0,0,0,0,t,-r,0,0,r,t,0,0,0,0,1),this}makeRotationY(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,0,r,0,0,1,0,0,-r,0,t,0,0,0,0,1),this}makeRotationZ(e){const t=Math.cos(e),r=Math.sin(e);return this.set(t,-r,0,0,r,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){const r=Math.cos(t),o=Math.sin(t),u=1-r,c=e.x,d=e.y,h=e.z,m=u*c,g=u*d;return this.set(m*c+r,m*d-o*h,m*h+o*d,0,m*d+o*h,g*d+r,g*h-o*c,0,m*h-o*d,g*h+o*c,u*h*h+r,0,0,0,0,1),this}makeScale(e,t,r){return this.set(e,0,0,0,0,t,0,0,0,0,r,0,0,0,0,1),this}makeShear(e,t,r,o,u,c){return this.set(1,r,u,0,e,1,c,0,t,o,1,0,0,0,0,1),this}compose(e,t,r){const o=this.elements,u=t._x,c=t._y,d=t._z,h=t._w,m=u+u,g=c+c,y=d+d,v=u*m,M=u*g,T=u*y,S=c*g,x=c*y,_=d*y,P=h*m,R=h*g,L=h*y,$=r.x,O=r.y,D=r.z;return o[0]=(1-(S+_))*$,o[1]=(M+L)*$,o[2]=(T-R)*$,o[3]=0,o[4]=(M-L)*O,o[5]=(1-(v+_))*O,o[6]=(x+P)*O,o[7]=0,o[8]=(T+R)*D,o[9]=(x-P)*D,o[10]=(1-(v+S))*D,o[11]=0,o[12]=e.x,o[13]=e.y,o[14]=e.z,o[15]=1,this}decompose(e,t,r){const o=this.elements;let u=Ls.set(o[0],o[1],o[2]).length();const c=Ls.set(o[4],o[5],o[6]).length(),d=Ls.set(o[8],o[9],o[10]).length();this.determinant()<0&&(u=-u),e.x=o[12],e.y=o[13],e.z=o[14],ai.copy(this);const m=1/u,g=1/c,y=1/d;return ai.elements[0]*=m,ai.elements[1]*=m,ai.elements[2]*=m,ai.elements[4]*=g,ai.elements[5]*=g,ai.elements[6]*=g,ai.elements[8]*=y,ai.elements[9]*=y,ai.elements[10]*=y,t.setFromRotationMatrix(ai),r.x=u,r.y=c,r.z=d,this}makePerspective(e,t,r,o,u,c,d=zi){const h=this.elements,m=2*u/(t-e),g=2*u/(r-o),y=(t+e)/(t-e),v=(r+o)/(r-o);let M,T;if(d===zi)M=-(c+u)/(c-u),T=-2*c*u/(c-u);else if(d===Kl)M=-c/(c-u),T=-c*u/(c-u);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+d);return h[0]=m,h[4]=0,h[8]=y,h[12]=0,h[1]=0,h[5]=g,h[9]=v,h[13]=0,h[2]=0,h[6]=0,h[10]=M,h[14]=T,h[3]=0,h[7]=0,h[11]=-1,h[15]=0,this}makeOrthographic(e,t,r,o,u,c,d=zi){const h=this.elements,m=1/(t-e),g=1/(r-o),y=1/(c-u),v=(t+e)*m,M=(r+o)*g;let T,S;if(d===zi)T=(c+u)*y,S=-2*y;else if(d===Kl)T=u*y,S=-1*y;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+d);return h[0]=2*m,h[4]=0,h[8]=0,h[12]=-v,h[1]=0,h[5]=2*g,h[9]=0,h[13]=-M,h[2]=0,h[6]=0,h[10]=S,h[14]=-T,h[3]=0,h[7]=0,h[11]=0,h[15]=1,this}equals(e){const t=this.elements,r=e.elements;for(let o=0;o<16;o++)if(t[o]!==r[o])return!1;return!0}fromArray(e,t=0){for(let r=0;r<16;r++)this.elements[r]=e[r+t];return this}toArray(e=[],t=0){const r=this.elements;return e[t]=r[0],e[t+1]=r[1],e[t+2]=r[2],e[t+3]=r[3],e[t+4]=r[4],e[t+5]=r[5],e[t+6]=r[6],e[t+7]=r[7],e[t+8]=r[8],e[t+9]=r[9],e[t+10]=r[10],e[t+11]=r[11],e[t+12]=r[12],e[t+13]=r[13],e[t+14]=r[14],e[t+15]=r[15],e}}const Ls=new J,ai=new Vt,sx=new J(0,0,0),ax=new J(1,1,1),mr=new J,gl=new J,kn=new J,Sm=new Vt,Mm=new to;class yi{constructor(e=0,t=0,r=0,o=yi.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=r,this._order=o}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,r,o=this._order){return this._x=e,this._y=t,this._z=r,this._order=o,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,r=!0){const o=e.elements,u=o[0],c=o[4],d=o[8],h=o[1],m=o[5],g=o[9],y=o[2],v=o[6],M=o[10];switch(t){case"XYZ":this._y=Math.asin(Mn(d,-1,1)),Math.abs(d)<.9999999?(this._x=Math.atan2(-g,M),this._z=Math.atan2(-c,u)):(this._x=Math.atan2(v,m),this._z=0);break;case"YXZ":this._x=Math.asin(-Mn(g,-1,1)),Math.abs(g)<.9999999?(this._y=Math.atan2(d,M),this._z=Math.atan2(h,m)):(this._y=Math.atan2(-y,u),this._z=0);break;case"ZXY":this._x=Math.asin(Mn(v,-1,1)),Math.abs(v)<.9999999?(this._y=Math.atan2(-y,M),this._z=Math.atan2(-c,m)):(this._y=0,this._z=Math.atan2(h,u));break;case"ZYX":this._y=Math.asin(-Mn(y,-1,1)),Math.abs(y)<.9999999?(this._x=Math.atan2(v,M),this._z=Math.atan2(h,u)):(this._x=0,this._z=Math.atan2(-c,m));break;case"YZX":this._z=Math.asin(Mn(h,-1,1)),Math.abs(h)<.9999999?(this._x=Math.atan2(-g,m),this._y=Math.atan2(-y,u)):(this._x=0,this._y=Math.atan2(d,M));break;case"XZY":this._z=Math.asin(-Mn(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(v,m),this._y=Math.atan2(d,u)):(this._x=Math.atan2(-g,M),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,r===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,r){return Sm.makeRotationFromQuaternion(e),this.setFromRotationMatrix(Sm,t,r)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return Mm.setFromEuler(this),this.setFromQuaternion(Mm,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}yi.DEFAULT_ORDER="XYZ";class Gg{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}}let ox=0;const Em=new J,Ns=new to,Ii=new Vt,_l=new J,Xa=new J,lx=new J,ux=new to,wm=new J(1,0,0),Tm=new J(0,1,0),Am=new J(0,0,1),Cm={type:"added"},cx={type:"removed"},Ds={type:"childadded",child:null},rf={type:"childremoved",child:null};class Qt extends aa{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:ox++}),this.uuid=Sr(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Qt.DEFAULT_UP.clone();const e=new J,t=new yi,r=new to,o=new J(1,1,1);function u(){r.setFromEuler(t,!1)}function c(){t.setFromQuaternion(r,void 0,!1)}t._onChange(u),r._onChange(c),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:r},scale:{configurable:!0,enumerable:!0,value:o},modelViewMatrix:{value:new Vt},normalMatrix:{value:new ht}}),this.matrix=new Vt,this.matrixWorld=new Vt,this.matrixAutoUpdate=Qt.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Qt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Gg,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return Ns.setFromAxisAngle(e,t),this.quaternion.multiply(Ns),this}rotateOnWorldAxis(e,t){return Ns.setFromAxisAngle(e,t),this.quaternion.premultiply(Ns),this}rotateX(e){return this.rotateOnAxis(wm,e)}rotateY(e){return this.rotateOnAxis(Tm,e)}rotateZ(e){return this.rotateOnAxis(Am,e)}translateOnAxis(e,t){return Em.copy(e).applyQuaternion(this.quaternion),this.position.add(Em.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(wm,e)}translateY(e){return this.translateOnAxis(Tm,e)}translateZ(e){return this.translateOnAxis(Am,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(Ii.copy(this.matrixWorld).invert())}lookAt(e,t,r){e.isVector3?_l.copy(e):_l.set(e,t,r);const o=this.parent;this.updateWorldMatrix(!0,!1),Xa.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Ii.lookAt(Xa,_l,this.up):Ii.lookAt(_l,Xa,this.up),this.quaternion.setFromRotationMatrix(Ii),o&&(Ii.extractRotation(o.matrixWorld),Ns.setFromRotationMatrix(Ii),this.quaternion.premultiply(Ns.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(Cm),Ds.child=e,this.dispatchEvent(Ds),Ds.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let r=0;r<arguments.length;r++)this.remove(arguments[r]);return this}const t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(cx),rf.child=e,this.dispatchEvent(rf),rf.child=null),this}removeFromParent(){const e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),Ii.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),Ii.multiply(e.parent.matrixWorld)),e.applyMatrix4(Ii),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(Cm),Ds.child=e,this.dispatchEvent(Ds),Ds.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let r=0,o=this.children.length;r<o;r++){const c=this.children[r].getObjectByProperty(e,t);if(c!==void 0)return c}}getObjectsByProperty(e,t,r=[]){this[e]===t&&r.push(this);const o=this.children;for(let u=0,c=o.length;u<c;u++)o[u].getObjectsByProperty(e,t,r);return r}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Xa,e,lx),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Xa,ux,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);const t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].traverseVisible(e)}traverseAncestors(e){const t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,e=!0);const t=this.children;for(let r=0,o=t.length;r<o;r++)t[r].updateMatrixWorld(e)}updateWorldMatrix(e,t){const r=this.parent;if(e===!0&&r!==null&&r.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),t===!0){const o=this.children;for(let u=0,c=o.length;u<c;u++)o[u].updateWorldMatrix(!1,!0)}}toJSON(e){const t=e===void 0||typeof e=="string",r={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},r.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const o={};o.uuid=this.uuid,o.type=this.type,this.name!==""&&(o.name=this.name),this.castShadow===!0&&(o.castShadow=!0),this.receiveShadow===!0&&(o.receiveShadow=!0),this.visible===!1&&(o.visible=!1),this.frustumCulled===!1&&(o.frustumCulled=!1),this.renderOrder!==0&&(o.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(o.userData=this.userData),o.layers=this.layers.mask,o.matrix=this.matrix.toArray(),o.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(o.matrixAutoUpdate=!1),this.isInstancedMesh&&(o.type="InstancedMesh",o.count=this.count,o.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(o.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(o.type="BatchedMesh",o.perObjectFrustumCulled=this.perObjectFrustumCulled,o.sortObjects=this.sortObjects,o.drawRanges=this._drawRanges,o.reservedRanges=this._reservedRanges,o.visibility=this._visibility,o.active=this._active,o.bounds=this._bounds.map(d=>({boxInitialized:d.boxInitialized,boxMin:d.box.min.toArray(),boxMax:d.box.max.toArray(),sphereInitialized:d.sphereInitialized,sphereRadius:d.sphere.radius,sphereCenter:d.sphere.center.toArray()})),o.maxInstanceCount=this._maxInstanceCount,o.maxVertexCount=this._maxVertexCount,o.maxIndexCount=this._maxIndexCount,o.geometryInitialized=this._geometryInitialized,o.geometryCount=this._geometryCount,o.matricesTexture=this._matricesTexture.toJSON(e),this._colorsTexture!==null&&(o.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(o.boundingSphere={center:o.boundingSphere.center.toArray(),radius:o.boundingSphere.radius}),this.boundingBox!==null&&(o.boundingBox={min:o.boundingBox.min.toArray(),max:o.boundingBox.max.toArray()}));function u(d,h){return d[h.uuid]===void 0&&(d[h.uuid]=h.toJSON(e)),h.uuid}if(this.isScene)this.background&&(this.background.isColor?o.background=this.background.toJSON():this.background.isTexture&&(o.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(o.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){o.geometry=u(e.geometries,this.geometry);const d=this.geometry.parameters;if(d!==void 0&&d.shapes!==void 0){const h=d.shapes;if(Array.isArray(h))for(let m=0,g=h.length;m<g;m++){const y=h[m];u(e.shapes,y)}else u(e.shapes,h)}}if(this.isSkinnedMesh&&(o.bindMode=this.bindMode,o.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(u(e.skeletons,this.skeleton),o.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const d=[];for(let h=0,m=this.material.length;h<m;h++)d.push(u(e.materials,this.material[h]));o.material=d}else o.material=u(e.materials,this.material);if(this.children.length>0){o.children=[];for(let d=0;d<this.children.length;d++)o.children.push(this.children[d].toJSON(e).object)}if(this.animations.length>0){o.animations=[];for(let d=0;d<this.animations.length;d++){const h=this.animations[d];o.animations.push(u(e.animations,h))}}if(t){const d=c(e.geometries),h=c(e.materials),m=c(e.textures),g=c(e.images),y=c(e.shapes),v=c(e.skeletons),M=c(e.animations),T=c(e.nodes);d.length>0&&(r.geometries=d),h.length>0&&(r.materials=h),m.length>0&&(r.textures=m),g.length>0&&(r.images=g),y.length>0&&(r.shapes=y),v.length>0&&(r.skeletons=v),M.length>0&&(r.animations=M),T.length>0&&(r.nodes=T)}return r.object=o,r;function c(d){const h=[];for(const m in d){const g=d[m];delete g.metadata,h.push(g)}return h}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let r=0;r<e.children.length;r++){const o=e.children[r];this.add(o.clone())}return this}}Qt.DEFAULT_UP=new J(0,1,0);Qt.DEFAULT_MATRIX_AUTO_UPDATE=!0;Qt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const oi=new J,Ui=new J,sf=new J,Fi=new J,Is=new J,Us=new J,Rm=new J,af=new J,of=new J,lf=new J;class ci{constructor(e=new J,t=new J,r=new J){this.a=e,this.b=t,this.c=r}static getNormal(e,t,r,o){o.subVectors(r,t),oi.subVectors(e,t),o.cross(oi);const u=o.lengthSq();return u>0?o.multiplyScalar(1/Math.sqrt(u)):o.set(0,0,0)}static getBarycoord(e,t,r,o,u){oi.subVectors(o,t),Ui.subVectors(r,t),sf.subVectors(e,t);const c=oi.dot(oi),d=oi.dot(Ui),h=oi.dot(sf),m=Ui.dot(Ui),g=Ui.dot(sf),y=c*m-d*d;if(y===0)return u.set(0,0,0),null;const v=1/y,M=(m*h-d*g)*v,T=(c*g-d*h)*v;return u.set(1-M-T,T,M)}static containsPoint(e,t,r,o){return this.getBarycoord(e,t,r,o,Fi)===null?!1:Fi.x>=0&&Fi.y>=0&&Fi.x+Fi.y<=1}static getInterpolation(e,t,r,o,u,c,d,h){return this.getBarycoord(e,t,r,o,Fi)===null?(h.x=0,h.y=0,"z"in h&&(h.z=0),"w"in h&&(h.w=0),null):(h.setScalar(0),h.addScaledVector(u,Fi.x),h.addScaledVector(c,Fi.y),h.addScaledVector(d,Fi.z),h)}static isFrontFacing(e,t,r,o){return oi.subVectors(r,t),Ui.subVectors(e,t),oi.cross(Ui).dot(o)<0}set(e,t,r){return this.a.copy(e),this.b.copy(t),this.c.copy(r),this}setFromPointsAndIndices(e,t,r,o){return this.a.copy(e[t]),this.b.copy(e[r]),this.c.copy(e[o]),this}setFromAttributeAndIndices(e,t,r,o){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,r),this.c.fromBufferAttribute(e,o),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return oi.subVectors(this.c,this.b),Ui.subVectors(this.a,this.b),oi.cross(Ui).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return ci.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return ci.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,r,o,u){return ci.getInterpolation(e,this.a,this.b,this.c,t,r,o,u)}containsPoint(e){return ci.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return ci.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){const r=this.a,o=this.b,u=this.c;let c,d;Is.subVectors(o,r),Us.subVectors(u,r),af.subVectors(e,r);const h=Is.dot(af),m=Us.dot(af);if(h<=0&&m<=0)return t.copy(r);of.subVectors(e,o);const g=Is.dot(of),y=Us.dot(of);if(g>=0&&y<=g)return t.copy(o);const v=h*y-g*m;if(v<=0&&h>=0&&g<=0)return c=h/(h-g),t.copy(r).addScaledVector(Is,c);lf.subVectors(e,u);const M=Is.dot(lf),T=Us.dot(lf);if(T>=0&&M<=T)return t.copy(u);const S=M*m-h*T;if(S<=0&&m>=0&&T<=0)return d=m/(m-T),t.copy(r).addScaledVector(Us,d);const x=g*T-M*y;if(x<=0&&y-g>=0&&M-T>=0)return Rm.subVectors(u,o),d=(y-g)/(y-g+(M-T)),t.copy(o).addScaledVector(Rm,d);const _=1/(x+S+v);return c=S*_,d=v*_,t.copy(r).addScaledVector(Is,c).addScaledVector(Us,d)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}}const Wg={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},gr={h:0,s:0,l:0},vl={h:0,s:0,l:0};function uf(s,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?s+(e-s)*6*t:t<1/2?e:t<2/3?s+(e-s)*6*(2/3-t):s}class _t{constructor(e,t,r){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,r)}set(e,t,r){if(t===void 0&&r===void 0){const o=e;o&&o.isColor?this.copy(o):typeof o=="number"?this.setHex(o):typeof o=="string"&&this.setStyle(o)}else this.setRGB(e,t,r);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=li){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,At.toWorkingColorSpace(this,t),this}setRGB(e,t,r,o=At.workingColorSpace){return this.r=e,this.g=t,this.b=r,At.toWorkingColorSpace(this,o),this}setHSL(e,t,r,o=At.workingColorSpace){if(e=$0(e,1),t=Mn(t,0,1),r=Mn(r,0,1),t===0)this.r=this.g=this.b=r;else{const u=r<=.5?r*(1+t):r+t-r*t,c=2*r-u;this.r=uf(c,u,e+1/3),this.g=uf(c,u,e),this.b=uf(c,u,e-1/3)}return At.toWorkingColorSpace(this,o),this}setStyle(e,t=li){function r(u){u!==void 0&&parseFloat(u)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let o;if(o=/^(\w+)\(([^\)]*)\)/.exec(e)){let u;const c=o[1],d=o[2];switch(c){case"rgb":case"rgba":if(u=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(d))return r(u[4]),this.setRGB(Math.min(255,parseInt(u[1],10))/255,Math.min(255,parseInt(u[2],10))/255,Math.min(255,parseInt(u[3],10))/255,t);if(u=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(d))return r(u[4]),this.setRGB(Math.min(100,parseInt(u[1],10))/100,Math.min(100,parseInt(u[2],10))/100,Math.min(100,parseInt(u[3],10))/100,t);break;case"hsl":case"hsla":if(u=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(d))return r(u[4]),this.setHSL(parseFloat(u[1])/360,parseFloat(u[2])/100,parseFloat(u[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(o=/^\#([A-Fa-f\d]+)$/.exec(e)){const u=o[1],c=u.length;if(c===3)return this.setRGB(parseInt(u.charAt(0),16)/15,parseInt(u.charAt(1),16)/15,parseInt(u.charAt(2),16)/15,t);if(c===6)return this.setHex(parseInt(u,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=li){const r=Wg[e.toLowerCase()];return r!==void 0?this.setHex(r,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=ea(e.r),this.g=ea(e.g),this.b=ea(e.b),this}copyLinearToSRGB(e){return this.r=$c(e.r),this.g=$c(e.g),this.b=$c(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=li){return At.fromWorkingColorSpace(mn.copy(this),e),Math.round(Mn(mn.r*255,0,255))*65536+Math.round(Mn(mn.g*255,0,255))*256+Math.round(Mn(mn.b*255,0,255))}getHexString(e=li){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=At.workingColorSpace){At.fromWorkingColorSpace(mn.copy(this),t);const r=mn.r,o=mn.g,u=mn.b,c=Math.max(r,o,u),d=Math.min(r,o,u);let h,m;const g=(d+c)/2;if(d===c)h=0,m=0;else{const y=c-d;switch(m=g<=.5?y/(c+d):y/(2-c-d),c){case r:h=(o-u)/y+(o<u?6:0);break;case o:h=(u-r)/y+2;break;case u:h=(r-o)/y+4;break}h/=6}return e.h=h,e.s=m,e.l=g,e}getRGB(e,t=At.workingColorSpace){return At.fromWorkingColorSpace(mn.copy(this),t),e.r=mn.r,e.g=mn.g,e.b=mn.b,e}getStyle(e=li){At.fromWorkingColorSpace(mn.copy(this),e);const t=mn.r,r=mn.g,o=mn.b;return e!==li?`color(${e} ${t.toFixed(3)} ${r.toFixed(3)} ${o.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(r*255)},${Math.round(o*255)})`}offsetHSL(e,t,r){return this.getHSL(gr),this.setHSL(gr.h+e,gr.s+t,gr.l+r)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,r){return this.r=e.r+(t.r-e.r)*r,this.g=e.g+(t.g-e.g)*r,this.b=e.b+(t.b-e.b)*r,this}lerpHSL(e,t){this.getHSL(gr),e.getHSL(vl);const r=Yc(gr.h,vl.h,t),o=Yc(gr.s,vl.s,t),u=Yc(gr.l,vl.l,t);return this.setHSL(r,o,u),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){const t=this.r,r=this.g,o=this.b,u=e.elements;return this.r=u[0]*t+u[3]*r+u[6]*o,this.g=u[1]*t+u[4]*r+u[7]*o,this.b=u[2]*t+u[5]*r+u[8]*o,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const mn=new _t;_t.NAMES=Wg;let fx=0;class ns extends aa{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:fx++}),this.uuid=Sr(),this.name="",this.type="Material",this.blending=Zs,this.side=Mr,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=Lf,this.blendDst=Nf,this.blendEquation=Yr,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new _t(0,0,0),this.blendAlpha=0,this.depthFunc=jl,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=mm,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=As,this.stencilZFail=As,this.stencilZPass=As,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(const t in e){const r=e[t];if(r===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}const o=this[t];if(o===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}o&&o.isColor?o.set(r):o&&o.isVector3&&r&&r.isVector3?o.copy(r):this[t]=r}}toJSON(e){const t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});const r={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};r.uuid=this.uuid,r.type=this.type,this.name!==""&&(r.name=this.name),this.color&&this.color.isColor&&(r.color=this.color.getHex()),this.roughness!==void 0&&(r.roughness=this.roughness),this.metalness!==void 0&&(r.metalness=this.metalness),this.sheen!==void 0&&(r.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(r.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(r.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(r.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(r.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(r.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(r.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(r.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(r.shininess=this.shininess),this.clearcoat!==void 0&&(r.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(r.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(r.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(r.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(r.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,r.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(r.dispersion=this.dispersion),this.iridescence!==void 0&&(r.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(r.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(r.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(r.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(r.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(r.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(r.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(r.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(r.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(r.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(r.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(r.lightMap=this.lightMap.toJSON(e).uuid,r.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(r.aoMap=this.aoMap.toJSON(e).uuid,r.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(r.bumpMap=this.bumpMap.toJSON(e).uuid,r.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(r.normalMap=this.normalMap.toJSON(e).uuid,r.normalMapType=this.normalMapType,r.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(r.displacementMap=this.displacementMap.toJSON(e).uuid,r.displacementScale=this.displacementScale,r.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(r.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(r.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(r.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(r.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(r.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(r.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(r.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(r.combine=this.combine)),this.envMapRotation!==void 0&&(r.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(r.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(r.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(r.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(r.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(r.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(r.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(r.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(r.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(r.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(r.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(r.size=this.size),this.shadowSide!==null&&(r.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(r.sizeAttenuation=this.sizeAttenuation),this.blending!==Zs&&(r.blending=this.blending),this.side!==Mr&&(r.side=this.side),this.vertexColors===!0&&(r.vertexColors=!0),this.opacity<1&&(r.opacity=this.opacity),this.transparent===!0&&(r.transparent=!0),this.blendSrc!==Lf&&(r.blendSrc=this.blendSrc),this.blendDst!==Nf&&(r.blendDst=this.blendDst),this.blendEquation!==Yr&&(r.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(r.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(r.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(r.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(r.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(r.blendAlpha=this.blendAlpha),this.depthFunc!==jl&&(r.depthFunc=this.depthFunc),this.depthTest===!1&&(r.depthTest=this.depthTest),this.depthWrite===!1&&(r.depthWrite=this.depthWrite),this.colorWrite===!1&&(r.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(r.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==mm&&(r.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(r.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(r.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==As&&(r.stencilFail=this.stencilFail),this.stencilZFail!==As&&(r.stencilZFail=this.stencilZFail),this.stencilZPass!==As&&(r.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(r.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(r.rotation=this.rotation),this.polygonOffset===!0&&(r.polygonOffset=!0),this.polygonOffsetFactor!==0&&(r.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(r.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(r.linewidth=this.linewidth),this.dashSize!==void 0&&(r.dashSize=this.dashSize),this.gapSize!==void 0&&(r.gapSize=this.gapSize),this.scale!==void 0&&(r.scale=this.scale),this.dithering===!0&&(r.dithering=!0),this.alphaTest>0&&(r.alphaTest=this.alphaTest),this.alphaHash===!0&&(r.alphaHash=!0),this.alphaToCoverage===!0&&(r.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(r.premultipliedAlpha=!0),this.forceSinglePass===!0&&(r.forceSinglePass=!0),this.wireframe===!0&&(r.wireframe=!0),this.wireframeLinewidth>1&&(r.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(r.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(r.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(r.flatShading=!0),this.visible===!1&&(r.visible=!1),this.toneMapped===!1&&(r.toneMapped=!1),this.fog===!1&&(r.fog=!1),Object.keys(this.userData).length>0&&(r.userData=this.userData);function o(u){const c=[];for(const d in u){const h=u[d];delete h.metadata,c.push(h)}return c}if(t){const u=o(e.textures),c=o(e.images);u.length>0&&(r.textures=u),c.length>0&&(r.images=c)}return r}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;const t=e.clippingPlanes;let r=null;if(t!==null){const o=t.length;r=new Array(o);for(let u=0;u!==o;++u)r[u]=t[u].clone()}return this.clippingPlanes=r,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}onBuild(){console.warn("Material: onBuild() has been removed.")}onBeforeRender(){console.warn("Material: onBeforeRender() has been removed.")}}class Ed extends ns{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new _t(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yi,this.combine=Tg,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}}const jt=new J,xl=new ft;class di{constructor(e,t,r=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=r,this.usage=ud,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=Bi,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return Js("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,r){e*=this.itemSize,r*=t.itemSize;for(let o=0,u=this.itemSize;o<u;o++)this.array[e+o]=t.array[r+o];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,r=this.count;t<r;t++)xl.fromBufferAttribute(this,t),xl.applyMatrix3(e),this.setXY(t,xl.x,xl.y);else if(this.itemSize===3)for(let t=0,r=this.count;t<r;t++)jt.fromBufferAttribute(this,t),jt.applyMatrix3(e),this.setXYZ(t,jt.x,jt.y,jt.z);return this}applyMatrix4(e){for(let t=0,r=this.count;t<r;t++)jt.fromBufferAttribute(this,t),jt.applyMatrix4(e),this.setXYZ(t,jt.x,jt.y,jt.z);return this}applyNormalMatrix(e){for(let t=0,r=this.count;t<r;t++)jt.fromBufferAttribute(this,t),jt.applyNormalMatrix(e),this.setXYZ(t,jt.x,jt.y,jt.z);return this}transformDirection(e){for(let t=0,r=this.count;t<r;t++)jt.fromBufferAttribute(this,t),jt.transformDirection(e),this.setXYZ(t,jt.x,jt.y,jt.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let r=this.array[e*this.itemSize+t];return this.normalized&&(r=xi(r,this.array)),r}setComponent(e,t,r){return this.normalized&&(r=Pt(r,this.array)),this.array[e*this.itemSize+t]=r,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=xi(t,this.array)),t}setX(e,t){return this.normalized&&(t=Pt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=xi(t,this.array)),t}setY(e,t){return this.normalized&&(t=Pt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=xi(t,this.array)),t}setZ(e,t){return this.normalized&&(t=Pt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=xi(t,this.array)),t}setW(e,t){return this.normalized&&(t=Pt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,r){return e*=this.itemSize,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array)),this.array[e+0]=t,this.array[e+1]=r,this}setXYZ(e,t,r,o){return e*=this.itemSize,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array),o=Pt(o,this.array)),this.array[e+0]=t,this.array[e+1]=r,this.array[e+2]=o,this}setXYZW(e,t,r,o,u){return e*=this.itemSize,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array),o=Pt(o,this.array),u=Pt(u,this.array)),this.array[e+0]=t,this.array[e+1]=r,this.array[e+2]=o,this.array[e+3]=u,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==ud&&(e.usage=this.usage),e}}class Xg extends di{constructor(e,t,r){super(new Uint16Array(e),t,r)}}class jg extends di{constructor(e,t,r){super(new Uint32Array(e),t,r)}}class _n extends di{constructor(e,t,r){super(new Float32Array(e),t,r)}}let dx=0;const qn=new Vt,cf=new Qt,Fs=new J,Bn=new no,ja=new no,on=new J;class Hn extends aa{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:dx++}),this.uuid=Sr(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(Bg(e)?jg:Xg)(e,1):this.index=e,this}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,r=0){this.groups.push({start:e,count:t,materialIndex:r})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){const t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);const r=this.attributes.normal;if(r!==void 0){const u=new ht().getNormalMatrix(e);r.applyNormalMatrix(u),r.needsUpdate=!0}const o=this.attributes.tangent;return o!==void 0&&(o.transformDirection(e),o.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return qn.makeRotationFromQuaternion(e),this.applyMatrix4(qn),this}rotateX(e){return qn.makeRotationX(e),this.applyMatrix4(qn),this}rotateY(e){return qn.makeRotationY(e),this.applyMatrix4(qn),this}rotateZ(e){return qn.makeRotationZ(e),this.applyMatrix4(qn),this}translate(e,t,r){return qn.makeTranslation(e,t,r),this.applyMatrix4(qn),this}scale(e,t,r){return qn.makeScale(e,t,r),this.applyMatrix4(qn),this}lookAt(e){return cf.lookAt(e),cf.updateMatrix(),this.applyMatrix4(cf.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Fs).negate(),this.translate(Fs.x,Fs.y,Fs.z),this}setFromPoints(e){const t=[];for(let r=0,o=e.length;r<o;r++){const u=e[r];t.push(u.x,u.y,u.z||0)}return this.setAttribute("position",new _n(t,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new no);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new J(-1/0,-1/0,-1/0),new J(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let r=0,o=t.length;r<o;r++){const u=t[r];Bn.setFromBufferAttribute(u),this.morphTargetsRelative?(on.addVectors(this.boundingBox.min,Bn.min),this.boundingBox.expandByPoint(on),on.addVectors(this.boundingBox.max,Bn.max),this.boundingBox.expandByPoint(on)):(this.boundingBox.expandByPoint(Bn.min),this.boundingBox.expandByPoint(Bn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new iu);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new J,1/0);return}if(e){const r=this.boundingSphere.center;if(Bn.setFromBufferAttribute(e),t)for(let u=0,c=t.length;u<c;u++){const d=t[u];ja.setFromBufferAttribute(d),this.morphTargetsRelative?(on.addVectors(Bn.min,ja.min),Bn.expandByPoint(on),on.addVectors(Bn.max,ja.max),Bn.expandByPoint(on)):(Bn.expandByPoint(ja.min),Bn.expandByPoint(ja.max))}Bn.getCenter(r);let o=0;for(let u=0,c=e.count;u<c;u++)on.fromBufferAttribute(e,u),o=Math.max(o,r.distanceToSquared(on));if(t)for(let u=0,c=t.length;u<c;u++){const d=t[u],h=this.morphTargetsRelative;for(let m=0,g=d.count;m<g;m++)on.fromBufferAttribute(d,m),h&&(Fs.fromBufferAttribute(e,m),on.add(Fs)),o=Math.max(o,r.distanceToSquared(on))}this.boundingSphere.radius=Math.sqrt(o),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const r=t.position,o=t.normal,u=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new di(new Float32Array(4*r.count),4));const c=this.getAttribute("tangent"),d=[],h=[];for(let j=0;j<r.count;j++)d[j]=new J,h[j]=new J;const m=new J,g=new J,y=new J,v=new ft,M=new ft,T=new ft,S=new J,x=new J;function _(j,b,w){m.fromBufferAttribute(r,j),g.fromBufferAttribute(r,b),y.fromBufferAttribute(r,w),v.fromBufferAttribute(u,j),M.fromBufferAttribute(u,b),T.fromBufferAttribute(u,w),g.sub(m),y.sub(m),M.sub(v),T.sub(v);const I=1/(M.x*T.y-T.x*M.y);isFinite(I)&&(S.copy(g).multiplyScalar(T.y).addScaledVector(y,-M.y).multiplyScalar(I),x.copy(y).multiplyScalar(M.x).addScaledVector(g,-T.x).multiplyScalar(I),d[j].add(S),d[b].add(S),d[w].add(S),h[j].add(x),h[b].add(x),h[w].add(x))}let P=this.groups;P.length===0&&(P=[{start:0,count:e.count}]);for(let j=0,b=P.length;j<b;++j){const w=P[j],I=w.start,Y=w.count;for(let K=I,oe=I+Y;K<oe;K+=3)_(e.getX(K+0),e.getX(K+1),e.getX(K+2))}const R=new J,L=new J,$=new J,O=new J;function D(j){$.fromBufferAttribute(o,j),O.copy($);const b=d[j];R.copy(b),R.sub($.multiplyScalar($.dot(b))).normalize(),L.crossVectors(O,b);const I=L.dot(h[j])<0?-1:1;c.setXYZW(j,R.x,R.y,R.z,I)}for(let j=0,b=P.length;j<b;++j){const w=P[j],I=w.start,Y=w.count;for(let K=I,oe=I+Y;K<oe;K+=3)D(e.getX(K+0)),D(e.getX(K+1)),D(e.getX(K+2))}}computeVertexNormals(){const e=this.index,t=this.getAttribute("position");if(t!==void 0){let r=this.getAttribute("normal");if(r===void 0)r=new di(new Float32Array(t.count*3),3),this.setAttribute("normal",r);else for(let v=0,M=r.count;v<M;v++)r.setXYZ(v,0,0,0);const o=new J,u=new J,c=new J,d=new J,h=new J,m=new J,g=new J,y=new J;if(e)for(let v=0,M=e.count;v<M;v+=3){const T=e.getX(v+0),S=e.getX(v+1),x=e.getX(v+2);o.fromBufferAttribute(t,T),u.fromBufferAttribute(t,S),c.fromBufferAttribute(t,x),g.subVectors(c,u),y.subVectors(o,u),g.cross(y),d.fromBufferAttribute(r,T),h.fromBufferAttribute(r,S),m.fromBufferAttribute(r,x),d.add(g),h.add(g),m.add(g),r.setXYZ(T,d.x,d.y,d.z),r.setXYZ(S,h.x,h.y,h.z),r.setXYZ(x,m.x,m.y,m.z)}else for(let v=0,M=t.count;v<M;v+=3)o.fromBufferAttribute(t,v+0),u.fromBufferAttribute(t,v+1),c.fromBufferAttribute(t,v+2),g.subVectors(c,u),y.subVectors(o,u),g.cross(y),r.setXYZ(v+0,g.x,g.y,g.z),r.setXYZ(v+1,g.x,g.y,g.z),r.setXYZ(v+2,g.x,g.y,g.z);this.normalizeNormals(),r.needsUpdate=!0}}normalizeNormals(){const e=this.attributes.normal;for(let t=0,r=e.count;t<r;t++)on.fromBufferAttribute(e,t),on.normalize(),e.setXYZ(t,on.x,on.y,on.z)}toNonIndexed(){function e(d,h){const m=d.array,g=d.itemSize,y=d.normalized,v=new m.constructor(h.length*g);let M=0,T=0;for(let S=0,x=h.length;S<x;S++){d.isInterleavedBufferAttribute?M=h[S]*d.data.stride+d.offset:M=h[S]*g;for(let _=0;_<g;_++)v[T++]=m[M++]}return new di(v,g,y)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const t=new Hn,r=this.index.array,o=this.attributes;for(const d in o){const h=o[d],m=e(h,r);t.setAttribute(d,m)}const u=this.morphAttributes;for(const d in u){const h=[],m=u[d];for(let g=0,y=m.length;g<y;g++){const v=m[g],M=e(v,r);h.push(M)}t.morphAttributes[d]=h}t.morphTargetsRelative=this.morphTargetsRelative;const c=this.groups;for(let d=0,h=c.length;d<h;d++){const m=c[d];t.addGroup(m.start,m.count,m.materialIndex)}return t}toJSON(){const e={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){const h=this.parameters;for(const m in h)h[m]!==void 0&&(e[m]=h[m]);return e}e.data={attributes:{}};const t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});const r=this.attributes;for(const h in r){const m=r[h];e.data.attributes[h]=m.toJSON(e.data)}const o={};let u=!1;for(const h in this.morphAttributes){const m=this.morphAttributes[h],g=[];for(let y=0,v=m.length;y<v;y++){const M=m[y];g.push(M.toJSON(e.data))}g.length>0&&(o[h]=g,u=!0)}u&&(e.data.morphAttributes=o,e.data.morphTargetsRelative=this.morphTargetsRelative);const c=this.groups;c.length>0&&(e.data.groups=JSON.parse(JSON.stringify(c)));const d=this.boundingSphere;return d!==null&&(e.data.boundingSphere={center:d.center.toArray(),radius:d.radius}),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const t={};this.name=e.name;const r=e.index;r!==null&&this.setIndex(r.clone(t));const o=e.attributes;for(const m in o){const g=o[m];this.setAttribute(m,g.clone(t))}const u=e.morphAttributes;for(const m in u){const g=[],y=u[m];for(let v=0,M=y.length;v<M;v++)g.push(y[v].clone(t));this.morphAttributes[m]=g}this.morphTargetsRelative=e.morphTargetsRelative;const c=e.groups;for(let m=0,g=c.length;m<g;m++){const y=c[m];this.addGroup(y.start,y.count,y.materialIndex)}const d=e.boundingBox;d!==null&&(this.boundingBox=d.clone());const h=e.boundingSphere;return h!==null&&(this.boundingSphere=h.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const bm=new Vt,Hr=new Vg,yl=new iu,Pm=new J,Os=new J,ks=new J,Bs=new J,ff=new J,Sl=new J,Ml=new ft,El=new ft,wl=new ft,Lm=new J,Nm=new J,Dm=new J,Tl=new J,Al=new J;class zn extends Qt{constructor(e=new Hn,t=new Ed){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){const t=this.geometry.morphAttributes,r=Object.keys(t);if(r.length>0){const o=t[r[0]];if(o!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let u=0,c=o.length;u<c;u++){const d=o[u].name||String(u);this.morphTargetInfluences.push(0),this.morphTargetDictionary[d]=u}}}}getVertexPosition(e,t){const r=this.geometry,o=r.attributes.position,u=r.morphAttributes.position,c=r.morphTargetsRelative;t.fromBufferAttribute(o,e);const d=this.morphTargetInfluences;if(u&&d){Sl.set(0,0,0);for(let h=0,m=u.length;h<m;h++){const g=d[h],y=u[h];g!==0&&(ff.fromBufferAttribute(y,e),c?Sl.addScaledVector(ff,g):Sl.addScaledVector(ff.sub(t),g))}t.add(Sl)}return t}raycast(e,t){const r=this.geometry,o=this.material,u=this.matrixWorld;o!==void 0&&(r.boundingSphere===null&&r.computeBoundingSphere(),yl.copy(r.boundingSphere),yl.applyMatrix4(u),Hr.copy(e.ray).recast(e.near),!(yl.containsPoint(Hr.origin)===!1&&(Hr.intersectSphere(yl,Pm)===null||Hr.origin.distanceToSquared(Pm)>(e.far-e.near)**2))&&(bm.copy(u).invert(),Hr.copy(e.ray).applyMatrix4(bm),!(r.boundingBox!==null&&Hr.intersectsBox(r.boundingBox)===!1)&&this._computeIntersections(e,t,Hr)))}_computeIntersections(e,t,r){let o;const u=this.geometry,c=this.material,d=u.index,h=u.attributes.position,m=u.attributes.uv,g=u.attributes.uv1,y=u.attributes.normal,v=u.groups,M=u.drawRange;if(d!==null)if(Array.isArray(c))for(let T=0,S=v.length;T<S;T++){const x=v[T],_=c[x.materialIndex],P=Math.max(x.start,M.start),R=Math.min(d.count,Math.min(x.start+x.count,M.start+M.count));for(let L=P,$=R;L<$;L+=3){const O=d.getX(L),D=d.getX(L+1),j=d.getX(L+2);o=Cl(this,_,e,r,m,g,y,O,D,j),o&&(o.faceIndex=Math.floor(L/3),o.face.materialIndex=x.materialIndex,t.push(o))}}else{const T=Math.max(0,M.start),S=Math.min(d.count,M.start+M.count);for(let x=T,_=S;x<_;x+=3){const P=d.getX(x),R=d.getX(x+1),L=d.getX(x+2);o=Cl(this,c,e,r,m,g,y,P,R,L),o&&(o.faceIndex=Math.floor(x/3),t.push(o))}}else if(h!==void 0)if(Array.isArray(c))for(let T=0,S=v.length;T<S;T++){const x=v[T],_=c[x.materialIndex],P=Math.max(x.start,M.start),R=Math.min(h.count,Math.min(x.start+x.count,M.start+M.count));for(let L=P,$=R;L<$;L+=3){const O=L,D=L+1,j=L+2;o=Cl(this,_,e,r,m,g,y,O,D,j),o&&(o.faceIndex=Math.floor(L/3),o.face.materialIndex=x.materialIndex,t.push(o))}}else{const T=Math.max(0,M.start),S=Math.min(h.count,M.start+M.count);for(let x=T,_=S;x<_;x+=3){const P=x,R=x+1,L=x+2;o=Cl(this,c,e,r,m,g,y,P,R,L),o&&(o.faceIndex=Math.floor(x/3),t.push(o))}}}}function hx(s,e,t,r,o,u,c,d){let h;if(e.side===Ln?h=r.intersectTriangle(c,u,o,!0,d):h=r.intersectTriangle(o,u,c,e.side===Mr,d),h===null)return null;Al.copy(d),Al.applyMatrix4(s.matrixWorld);const m=t.ray.origin.distanceTo(Al);return m<t.near||m>t.far?null:{distance:m,point:Al.clone(),object:s}}function Cl(s,e,t,r,o,u,c,d,h,m){s.getVertexPosition(d,Os),s.getVertexPosition(h,ks),s.getVertexPosition(m,Bs);const g=hx(s,e,t,r,Os,ks,Bs,Tl);if(g){o&&(Ml.fromBufferAttribute(o,d),El.fromBufferAttribute(o,h),wl.fromBufferAttribute(o,m),g.uv=ci.getInterpolation(Tl,Os,ks,Bs,Ml,El,wl,new ft)),u&&(Ml.fromBufferAttribute(u,d),El.fromBufferAttribute(u,h),wl.fromBufferAttribute(u,m),g.uv1=ci.getInterpolation(Tl,Os,ks,Bs,Ml,El,wl,new ft)),c&&(Lm.fromBufferAttribute(c,d),Nm.fromBufferAttribute(c,h),Dm.fromBufferAttribute(c,m),g.normal=ci.getInterpolation(Tl,Os,ks,Bs,Lm,Nm,Dm,new J),g.normal.dot(r.direction)>0&&g.normal.multiplyScalar(-1));const y={a:d,b:h,c:m,normal:new J,materialIndex:0};ci.getNormal(Os,ks,Bs,y.normal),g.face=y}return g}class ts extends Hn{constructor(e=1,t=1,r=1,o=1,u=1,c=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:r,widthSegments:o,heightSegments:u,depthSegments:c};const d=this;o=Math.floor(o),u=Math.floor(u),c=Math.floor(c);const h=[],m=[],g=[],y=[];let v=0,M=0;T("z","y","x",-1,-1,r,t,e,c,u,0),T("z","y","x",1,-1,r,t,-e,c,u,1),T("x","z","y",1,1,e,r,t,o,c,2),T("x","z","y",1,-1,e,r,-t,o,c,3),T("x","y","z",1,-1,e,t,r,o,u,4),T("x","y","z",-1,-1,e,t,-r,o,u,5),this.setIndex(h),this.setAttribute("position",new _n(m,3)),this.setAttribute("normal",new _n(g,3)),this.setAttribute("uv",new _n(y,2));function T(S,x,_,P,R,L,$,O,D,j,b){const w=L/D,I=$/j,Y=L/2,K=$/2,oe=O/2,ne=D+1,B=j+1;let G=0,k=0;const ue=new J;for(let le=0;le<B;le++){const F=le*I-K;for(let ce=0;ce<ne;ce++){const Ie=ce*w-Y;ue[S]=Ie*P,ue[x]=F*R,ue[_]=oe,m.push(ue.x,ue.y,ue.z),ue[S]=0,ue[x]=0,ue[_]=O>0?1:-1,g.push(ue.x,ue.y,ue.z),y.push(ce/D),y.push(1-le/j),G+=1}}for(let le=0;le<j;le++)for(let F=0;F<D;F++){const ce=v+F+ne*le,Ie=v+F+ne*(le+1),te=v+(F+1)+ne*(le+1),fe=v+(F+1)+ne*le;h.push(ce,Ie,fe),h.push(Ie,te,fe),k+=6}d.addGroup(M,k,b),M+=k,v+=G}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new ts(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}}function sa(s){const e={};for(const t in s){e[t]={};for(const r in s[t]){const o=s[t][r];o&&(o.isColor||o.isMatrix3||o.isMatrix4||o.isVector2||o.isVector3||o.isVector4||o.isTexture||o.isQuaternion)?o.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][r]=null):e[t][r]=o.clone():Array.isArray(o)?e[t][r]=o.slice():e[t][r]=o}}return e}function Sn(s){const e={};for(let t=0;t<s.length;t++){const r=sa(s[t]);for(const o in r)e[o]=r[o]}return e}function px(s){const e=[];for(let t=0;t<s.length;t++)e.push(s[t].clone());return e}function Yg(s){const e=s.getRenderTarget();return e===null?s.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:At.workingColorSpace}const mx={clone:sa,merge:Sn};var gx=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,_x=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Er extends ns{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=gx,this.fragmentShader=_x,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=sa(e.uniforms),this.uniformsGroups=px(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){const t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(const o in this.uniforms){const c=this.uniforms[o].value;c&&c.isTexture?t.uniforms[o]={type:"t",value:c.toJSON(e).uuid}:c&&c.isColor?t.uniforms[o]={type:"c",value:c.getHex()}:c&&c.isVector2?t.uniforms[o]={type:"v2",value:c.toArray()}:c&&c.isVector3?t.uniforms[o]={type:"v3",value:c.toArray()}:c&&c.isVector4?t.uniforms[o]={type:"v4",value:c.toArray()}:c&&c.isMatrix3?t.uniforms[o]={type:"m3",value:c.toArray()}:c&&c.isMatrix4?t.uniforms[o]={type:"m4",value:c.toArray()}:t.uniforms[o]={value:c}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;const r={};for(const o in this.extensions)this.extensions[o]===!0&&(r[o]=!0);return Object.keys(r).length>0&&(t.extensions=r),t}}class qg extends Qt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new Vt,this.projectionMatrix=new Vt,this.projectionMatrixInverse=new Vt,this.coordinateSystem=zi}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const _r=new J,Im=new ft,Um=new ft;class $n extends qg{constructor(e=50,t=1,r=.1,o=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=r,this.far=o,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){const t=.5*this.getFilmHeight()/e;this.fov=cd*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){const e=Math.tan(jc*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return cd*2*Math.atan(Math.tan(jc*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,r){_r.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(_r.x,_r.y).multiplyScalar(-e/_r.z),_r.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),r.set(_r.x,_r.y).multiplyScalar(-e/_r.z)}getViewSize(e,t){return this.getViewBounds(e,Im,Um),t.subVectors(Um,Im)}setViewOffset(e,t,r,o,u,c){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=r,this.view.offsetY=o,this.view.width=u,this.view.height=c,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=this.near;let t=e*Math.tan(jc*.5*this.fov)/this.zoom,r=2*t,o=this.aspect*r,u=-.5*o;const c=this.view;if(this.view!==null&&this.view.enabled){const h=c.fullWidth,m=c.fullHeight;u+=c.offsetX*o/h,t-=c.offsetY*r/m,o*=c.width/h,r*=c.height/m}const d=this.filmOffset;d!==0&&(u+=e*d/this.getFilmWidth()),this.projectionMatrix.makePerspective(u,u+o,t,t-r,e,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}}const zs=-90,Hs=1;class vx extends Qt{constructor(e,t,r){super(),this.type="CubeCamera",this.renderTarget=r,this.coordinateSystem=null,this.activeMipmapLevel=0;const o=new $n(zs,Hs,e,t);o.layers=this.layers,this.add(o);const u=new $n(zs,Hs,e,t);u.layers=this.layers,this.add(u);const c=new $n(zs,Hs,e,t);c.layers=this.layers,this.add(c);const d=new $n(zs,Hs,e,t);d.layers=this.layers,this.add(d);const h=new $n(zs,Hs,e,t);h.layers=this.layers,this.add(h);const m=new $n(zs,Hs,e,t);m.layers=this.layers,this.add(m)}updateCoordinateSystem(){const e=this.coordinateSystem,t=this.children.concat(),[r,o,u,c,d,h]=t;for(const m of t)this.remove(m);if(e===zi)r.up.set(0,1,0),r.lookAt(1,0,0),o.up.set(0,1,0),o.lookAt(-1,0,0),u.up.set(0,0,-1),u.lookAt(0,1,0),c.up.set(0,0,1),c.lookAt(0,-1,0),d.up.set(0,1,0),d.lookAt(0,0,1),h.up.set(0,1,0),h.lookAt(0,0,-1);else if(e===Kl)r.up.set(0,-1,0),r.lookAt(-1,0,0),o.up.set(0,-1,0),o.lookAt(1,0,0),u.up.set(0,0,1),u.lookAt(0,1,0),c.up.set(0,0,-1),c.lookAt(0,-1,0),d.up.set(0,-1,0),d.lookAt(0,0,1),h.up.set(0,-1,0),h.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(const m of t)this.add(m),m.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();const{renderTarget:r,activeMipmapLevel:o}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());const[u,c,d,h,m,g]=this.children,y=e.getRenderTarget(),v=e.getActiveCubeFace(),M=e.getActiveMipmapLevel(),T=e.xr.enabled;e.xr.enabled=!1;const S=r.texture.generateMipmaps;r.texture.generateMipmaps=!1,e.setRenderTarget(r,0,o),e.render(t,u),e.setRenderTarget(r,1,o),e.render(t,c),e.setRenderTarget(r,2,o),e.render(t,d),e.setRenderTarget(r,3,o),e.render(t,h),e.setRenderTarget(r,4,o),e.render(t,m),r.texture.generateMipmaps=S,e.setRenderTarget(r,5,o),e.render(t,g),e.setRenderTarget(y,v,M),e.xr.enabled=T,r.texture.needsPMREMUpdate=!0}}class $g extends En{constructor(e,t,r,o,u,c,d,h,m,g){e=e!==void 0?e:[],t=t!==void 0?t:ta,super(e,t,r,o,u,c,d,h,m,g),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}}class xx extends es{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;const r={width:e,height:e,depth:1},o=[r,r,r,r,r,r];this.texture=new $g(o,t.mapping,t.wrapS,t.wrapT,t.magFilter,t.minFilter,t.format,t.type,t.anisotropy,t.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=t.generateMipmaps!==void 0?t.generateMipmaps:!1,this.texture.minFilter=t.minFilter!==void 0?t.minFilter:ui}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;const r={uniforms:{tEquirect:{value:null}},vertexShader:`

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
			`},o=new ts(5,5,5),u=new Er({name:"CubemapFromEquirect",uniforms:sa(r.uniforms),vertexShader:r.vertexShader,fragmentShader:r.fragmentShader,side:Ln,blending:xr});u.uniforms.tEquirect.value=t;const c=new zn(o,u),d=t.minFilter;return t.minFilter===Kr&&(t.minFilter=ui),new vx(1,10,this).update(e,c),t.minFilter=d,c.geometry.dispose(),c.material.dispose(),this}clear(e,t,r,o){const u=e.getRenderTarget();for(let c=0;c<6;c++)e.setRenderTarget(this,c),e.clear(t,r,o);e.setRenderTarget(u)}}const df=new J,yx=new J,Sx=new ht;class Xr{constructor(e=new J(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,r,o){return this.normal.set(e,t,r),this.constant=o,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,r){const o=df.subVectors(r,t).cross(yx.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(o,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){const e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){const r=e.delta(df),o=this.normal.dot(r);if(o===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;const u=-(e.start.dot(this.normal)+this.constant)/o;return u<0||u>1?null:t.copy(e.start).addScaledVector(r,u)}intersectsLine(e){const t=this.distanceToPoint(e.start),r=this.distanceToPoint(e.end);return t<0&&r>0||r<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){const r=t||Sx.getNormalMatrix(e),o=this.coplanarPoint(df).applyMatrix4(e),u=this.normal.applyMatrix3(r).normalize();return this.constant=-o.dot(u),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Vr=new iu,Rl=new J;class wd{constructor(e=new Xr,t=new Xr,r=new Xr,o=new Xr,u=new Xr,c=new Xr){this.planes=[e,t,r,o,u,c]}set(e,t,r,o,u,c){const d=this.planes;return d[0].copy(e),d[1].copy(t),d[2].copy(r),d[3].copy(o),d[4].copy(u),d[5].copy(c),this}copy(e){const t=this.planes;for(let r=0;r<6;r++)t[r].copy(e.planes[r]);return this}setFromProjectionMatrix(e,t=zi){const r=this.planes,o=e.elements,u=o[0],c=o[1],d=o[2],h=o[3],m=o[4],g=o[5],y=o[6],v=o[7],M=o[8],T=o[9],S=o[10],x=o[11],_=o[12],P=o[13],R=o[14],L=o[15];if(r[0].setComponents(h-u,v-m,x-M,L-_).normalize(),r[1].setComponents(h+u,v+m,x+M,L+_).normalize(),r[2].setComponents(h+c,v+g,x+T,L+P).normalize(),r[3].setComponents(h-c,v-g,x-T,L-P).normalize(),r[4].setComponents(h-d,v-y,x-S,L-R).normalize(),t===zi)r[5].setComponents(h+d,v+y,x+S,L+R).normalize();else if(t===Kl)r[5].setComponents(d,y,S,R).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),Vr.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{const t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),Vr.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(Vr)}intersectsSprite(e){return Vr.center.set(0,0,0),Vr.radius=.7071067811865476,Vr.applyMatrix4(e.matrixWorld),this.intersectsSphere(Vr)}intersectsSphere(e){const t=this.planes,r=e.center,o=-e.radius;for(let u=0;u<6;u++)if(t[u].distanceToPoint(r)<o)return!1;return!0}intersectsBox(e){const t=this.planes;for(let r=0;r<6;r++){const o=t[r];if(Rl.x=o.normal.x>0?e.max.x:e.min.x,Rl.y=o.normal.y>0?e.max.y:e.min.y,Rl.z=o.normal.z>0?e.max.z:e.min.z,o.distanceToPoint(Rl)<0)return!1}return!0}containsPoint(e){const t=this.planes;for(let r=0;r<6;r++)if(t[r].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function Kg(){let s=null,e=!1,t=null,r=null;function o(u,c){t(u,c),r=s.requestAnimationFrame(o)}return{start:function(){e!==!0&&t!==null&&(r=s.requestAnimationFrame(o),e=!0)},stop:function(){s.cancelAnimationFrame(r),e=!1},setAnimationLoop:function(u){t=u},setContext:function(u){s=u}}}function Mx(s){const e=new WeakMap;function t(d,h){const m=d.array,g=d.usage,y=m.byteLength,v=s.createBuffer();s.bindBuffer(h,v),s.bufferData(h,m,g),d.onUploadCallback();let M;if(m instanceof Float32Array)M=s.FLOAT;else if(m instanceof Uint16Array)d.isFloat16BufferAttribute?M=s.HALF_FLOAT:M=s.UNSIGNED_SHORT;else if(m instanceof Int16Array)M=s.SHORT;else if(m instanceof Uint32Array)M=s.UNSIGNED_INT;else if(m instanceof Int32Array)M=s.INT;else if(m instanceof Int8Array)M=s.BYTE;else if(m instanceof Uint8Array)M=s.UNSIGNED_BYTE;else if(m instanceof Uint8ClampedArray)M=s.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+m);return{buffer:v,type:M,bytesPerElement:m.BYTES_PER_ELEMENT,version:d.version,size:y}}function r(d,h,m){const g=h.array,y=h._updateRange,v=h.updateRanges;if(s.bindBuffer(m,d),y.count===-1&&v.length===0&&s.bufferSubData(m,0,g),v.length!==0){for(let M=0,T=v.length;M<T;M++){const S=v[M];s.bufferSubData(m,S.start*g.BYTES_PER_ELEMENT,g,S.start,S.count)}h.clearUpdateRanges()}y.count!==-1&&(s.bufferSubData(m,y.offset*g.BYTES_PER_ELEMENT,g,y.offset,y.count),y.count=-1),h.onUploadCallback()}function o(d){return d.isInterleavedBufferAttribute&&(d=d.data),e.get(d)}function u(d){d.isInterleavedBufferAttribute&&(d=d.data);const h=e.get(d);h&&(s.deleteBuffer(h.buffer),e.delete(d))}function c(d,h){if(d.isInterleavedBufferAttribute&&(d=d.data),d.isGLBufferAttribute){const g=e.get(d);(!g||g.version<d.version)&&e.set(d,{buffer:d.buffer,type:d.type,bytesPerElement:d.elementSize,version:d.version});return}const m=e.get(d);if(m===void 0)e.set(d,t(d,h));else if(m.version<d.version){if(m.size!==d.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");r(m.buffer,d,h),m.version=d.version}}return{get:o,remove:u,update:c}}class ru extends Hn{constructor(e=1,t=1,r=1,o=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:r,heightSegments:o};const u=e/2,c=t/2,d=Math.floor(r),h=Math.floor(o),m=d+1,g=h+1,y=e/d,v=t/h,M=[],T=[],S=[],x=[];for(let _=0;_<g;_++){const P=_*v-c;for(let R=0;R<m;R++){const L=R*y-u;T.push(L,-P,0),S.push(0,0,1),x.push(R/d),x.push(1-_/h)}}for(let _=0;_<h;_++)for(let P=0;P<d;P++){const R=P+m*_,L=P+m*(_+1),$=P+1+m*(_+1),O=P+1+m*_;M.push(R,L,O),M.push(L,$,O)}this.setIndex(M),this.setAttribute("position",new _n(T,3)),this.setAttribute("normal",new _n(S,3)),this.setAttribute("uv",new _n(x,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new ru(e.width,e.height,e.widthSegments,e.heightSegments)}}var Ex=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,wx=`#ifdef USE_ALPHAHASH
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
#endif`,Tx=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,Ax=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Cx=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,Rx=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,bx=`#ifdef USE_AOMAP
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
#endif`,Px=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Lx=`#ifdef USE_BATCHING
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
#endif`,Nx=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,Dx=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Ix=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Ux=`float G_BlinnPhong_Implicit( ) {
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
} // validated`,Fx=`#ifdef USE_IRIDESCENCE
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
#endif`,Ox=`#ifdef USE_BUMPMAP
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
#endif`,kx=`#if NUM_CLIPPING_PLANES > 0
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
#endif`,Bx=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,zx=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Hx=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,Vx=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Gx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Wx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,Xx=`#if defined( USE_COLOR_ALPHA )
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
#endif`,jx=`#define PI 3.141592653589793
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
} // validated`,Yx=`#ifdef ENVMAP_TYPE_CUBE_UV
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
#endif`,qx=`vec3 transformedNormal = objectNormal;
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
#endif`,$x=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,Kx=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Zx=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,Qx=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,Jx="gl_FragColor = linearToOutputTexel( gl_FragColor );",ey=`
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
}`,ty=`#ifdef USE_ENVMAP
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
#endif`,ny=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,iy=`#ifdef USE_ENVMAP
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
#endif`,ry=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,sy=`#ifdef USE_ENVMAP
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
#endif`,ay=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,oy=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,ly=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,uy=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,cy=`#ifdef USE_GRADIENTMAP
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
}`,fy=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,dy=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,hy=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,py=`uniform bool receiveShadow;
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
#endif`,my=`#ifdef USE_ENVMAP
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
#endif`,gy=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,_y=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,vy=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,xy=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,yy=`PhysicalMaterial material;
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
#endif`,Sy=`struct PhysicalMaterial {
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
}`,My=`
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
#endif`,Ey=`#if defined( RE_IndirectDiffuse )
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
#endif`,wy=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Ty=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,Ay=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Cy=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Ry=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,by=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Py=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,Ly=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
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
#endif`,Ny=`#if defined( USE_POINTS_UV )
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
#endif`,Dy=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,Iy=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Uy=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,Fy=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,Oy=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,ky=`#ifdef USE_MORPHTARGETS
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
#endif`,By=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,zy=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
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
vec3 nonPerturbedNormal = normal;`,Hy=`#ifdef USE_NORMALMAP_OBJECTSPACE
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
#endif`,Vy=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Gy=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Wy=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,Xy=`#ifdef USE_NORMALMAP
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
#endif`,jy=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,Yy=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,qy=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,$y=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,Ky=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,Zy=`vec3 packNormalToRGB( const in vec3 normal ) {
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
}`,Qy=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,Jy=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,eS=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,tS=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,nS=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,iS=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,rS=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,sS=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,aS=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
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
#endif`,oS=`float getShadowMask() {
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
}`,lS=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,uS=`#ifdef USE_SKINNING
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
#endif`,cS=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,fS=`#ifdef USE_SKINNING
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
#endif`,dS=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,hS=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,pS=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,mS=`#ifndef saturate
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
vec3 CustomToneMapping( vec3 color ) { return color; }`,gS=`#ifdef USE_TRANSMISSION
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
#endif`,_S=`#ifdef USE_TRANSMISSION
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
#endif`,vS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,xS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,yS=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,SS=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const MS=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,ES=`uniform sampler2D t2D;
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
}`,wS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,TS=`#ifdef ENVMAP_TYPE_CUBE
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
}`,AS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,CS=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,RS=`#include <common>
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
}`,bS=`#if DEPTH_PACKING == 3200
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
}`,PS=`#define DISTANCE
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
}`,LS=`#define DISTANCE
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
}`,NS=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,DS=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,IS=`uniform float scale;
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
}`,US=`uniform vec3 diffuse;
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
}`,FS=`#include <common>
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
}`,OS=`uniform vec3 diffuse;
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
}`,kS=`#define LAMBERT
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
}`,BS=`#define LAMBERT
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
}`,zS=`#define MATCAP
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
}`,HS=`#define MATCAP
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
}`,VS=`#define NORMAL
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
}`,GS=`#define NORMAL
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
}`,WS=`#define PHONG
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
}`,XS=`#define PHONG
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
}`,jS=`#define STANDARD
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
}`,YS=`#define STANDARD
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
}`,qS=`#define TOON
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
}`,$S=`#define TOON
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
}`,KS=`uniform float size;
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
}`,ZS=`uniform vec3 diffuse;
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
}`,QS=`#include <common>
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
}`,JS=`uniform vec3 color;
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
}`,eM=`uniform float rotation;
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
}`,tM=`uniform vec3 diffuse;
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
}`,dt={alphahash_fragment:Ex,alphahash_pars_fragment:wx,alphamap_fragment:Tx,alphamap_pars_fragment:Ax,alphatest_fragment:Cx,alphatest_pars_fragment:Rx,aomap_fragment:bx,aomap_pars_fragment:Px,batching_pars_vertex:Lx,batching_vertex:Nx,begin_vertex:Dx,beginnormal_vertex:Ix,bsdfs:Ux,iridescence_fragment:Fx,bumpmap_pars_fragment:Ox,clipping_planes_fragment:kx,clipping_planes_pars_fragment:Bx,clipping_planes_pars_vertex:zx,clipping_planes_vertex:Hx,color_fragment:Vx,color_pars_fragment:Gx,color_pars_vertex:Wx,color_vertex:Xx,common:jx,cube_uv_reflection_fragment:Yx,defaultnormal_vertex:qx,displacementmap_pars_vertex:$x,displacementmap_vertex:Kx,emissivemap_fragment:Zx,emissivemap_pars_fragment:Qx,colorspace_fragment:Jx,colorspace_pars_fragment:ey,envmap_fragment:ty,envmap_common_pars_fragment:ny,envmap_pars_fragment:iy,envmap_pars_vertex:ry,envmap_physical_pars_fragment:my,envmap_vertex:sy,fog_vertex:ay,fog_pars_vertex:oy,fog_fragment:ly,fog_pars_fragment:uy,gradientmap_pars_fragment:cy,lightmap_pars_fragment:fy,lights_lambert_fragment:dy,lights_lambert_pars_fragment:hy,lights_pars_begin:py,lights_toon_fragment:gy,lights_toon_pars_fragment:_y,lights_phong_fragment:vy,lights_phong_pars_fragment:xy,lights_physical_fragment:yy,lights_physical_pars_fragment:Sy,lights_fragment_begin:My,lights_fragment_maps:Ey,lights_fragment_end:wy,logdepthbuf_fragment:Ty,logdepthbuf_pars_fragment:Ay,logdepthbuf_pars_vertex:Cy,logdepthbuf_vertex:Ry,map_fragment:by,map_pars_fragment:Py,map_particle_fragment:Ly,map_particle_pars_fragment:Ny,metalnessmap_fragment:Dy,metalnessmap_pars_fragment:Iy,morphinstance_vertex:Uy,morphcolor_vertex:Fy,morphnormal_vertex:Oy,morphtarget_pars_vertex:ky,morphtarget_vertex:By,normal_fragment_begin:zy,normal_fragment_maps:Hy,normal_pars_fragment:Vy,normal_pars_vertex:Gy,normal_vertex:Wy,normalmap_pars_fragment:Xy,clearcoat_normal_fragment_begin:jy,clearcoat_normal_fragment_maps:Yy,clearcoat_pars_fragment:qy,iridescence_pars_fragment:$y,opaque_fragment:Ky,packing:Zy,premultiplied_alpha_fragment:Qy,project_vertex:Jy,dithering_fragment:eS,dithering_pars_fragment:tS,roughnessmap_fragment:nS,roughnessmap_pars_fragment:iS,shadowmap_pars_fragment:rS,shadowmap_pars_vertex:sS,shadowmap_vertex:aS,shadowmask_pars_fragment:oS,skinbase_vertex:lS,skinning_pars_vertex:uS,skinning_vertex:cS,skinnormal_vertex:fS,specularmap_fragment:dS,specularmap_pars_fragment:hS,tonemapping_fragment:pS,tonemapping_pars_fragment:mS,transmission_fragment:gS,transmission_pars_fragment:_S,uv_pars_fragment:vS,uv_pars_vertex:xS,uv_vertex:yS,worldpos_vertex:SS,background_vert:MS,background_frag:ES,backgroundCube_vert:wS,backgroundCube_frag:TS,cube_vert:AS,cube_frag:CS,depth_vert:RS,depth_frag:bS,distanceRGBA_vert:PS,distanceRGBA_frag:LS,equirect_vert:NS,equirect_frag:DS,linedashed_vert:IS,linedashed_frag:US,meshbasic_vert:FS,meshbasic_frag:OS,meshlambert_vert:kS,meshlambert_frag:BS,meshmatcap_vert:zS,meshmatcap_frag:HS,meshnormal_vert:VS,meshnormal_frag:GS,meshphong_vert:WS,meshphong_frag:XS,meshphysical_vert:jS,meshphysical_frag:YS,meshtoon_vert:qS,meshtoon_frag:$S,points_vert:KS,points_frag:ZS,shadow_vert:QS,shadow_frag:JS,sprite_vert:eM,sprite_frag:tM},Pe={common:{diffuse:{value:new _t(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ht},alphaMap:{value:null},alphaMapTransform:{value:new ht},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ht}},envmap:{envMap:{value:null},envMapRotation:{value:new ht},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ht}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ht}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ht},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ht},normalScale:{value:new ft(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ht},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ht}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ht}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ht}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new _t(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new _t(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ht},alphaTest:{value:0},uvTransform:{value:new ht}},sprite:{diffuse:{value:new _t(16777215)},opacity:{value:1},center:{value:new ft(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ht},alphaMap:{value:null},alphaMapTransform:{value:new ht},alphaTest:{value:0}}},vi={basic:{uniforms:Sn([Pe.common,Pe.specularmap,Pe.envmap,Pe.aomap,Pe.lightmap,Pe.fog]),vertexShader:dt.meshbasic_vert,fragmentShader:dt.meshbasic_frag},lambert:{uniforms:Sn([Pe.common,Pe.specularmap,Pe.envmap,Pe.aomap,Pe.lightmap,Pe.emissivemap,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,Pe.fog,Pe.lights,{emissive:{value:new _t(0)}}]),vertexShader:dt.meshlambert_vert,fragmentShader:dt.meshlambert_frag},phong:{uniforms:Sn([Pe.common,Pe.specularmap,Pe.envmap,Pe.aomap,Pe.lightmap,Pe.emissivemap,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,Pe.fog,Pe.lights,{emissive:{value:new _t(0)},specular:{value:new _t(1118481)},shininess:{value:30}}]),vertexShader:dt.meshphong_vert,fragmentShader:dt.meshphong_frag},standard:{uniforms:Sn([Pe.common,Pe.envmap,Pe.aomap,Pe.lightmap,Pe.emissivemap,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,Pe.roughnessmap,Pe.metalnessmap,Pe.fog,Pe.lights,{emissive:{value:new _t(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:dt.meshphysical_vert,fragmentShader:dt.meshphysical_frag},toon:{uniforms:Sn([Pe.common,Pe.aomap,Pe.lightmap,Pe.emissivemap,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,Pe.gradientmap,Pe.fog,Pe.lights,{emissive:{value:new _t(0)}}]),vertexShader:dt.meshtoon_vert,fragmentShader:dt.meshtoon_frag},matcap:{uniforms:Sn([Pe.common,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,Pe.fog,{matcap:{value:null}}]),vertexShader:dt.meshmatcap_vert,fragmentShader:dt.meshmatcap_frag},points:{uniforms:Sn([Pe.points,Pe.fog]),vertexShader:dt.points_vert,fragmentShader:dt.points_frag},dashed:{uniforms:Sn([Pe.common,Pe.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:dt.linedashed_vert,fragmentShader:dt.linedashed_frag},depth:{uniforms:Sn([Pe.common,Pe.displacementmap]),vertexShader:dt.depth_vert,fragmentShader:dt.depth_frag},normal:{uniforms:Sn([Pe.common,Pe.bumpmap,Pe.normalmap,Pe.displacementmap,{opacity:{value:1}}]),vertexShader:dt.meshnormal_vert,fragmentShader:dt.meshnormal_frag},sprite:{uniforms:Sn([Pe.sprite,Pe.fog]),vertexShader:dt.sprite_vert,fragmentShader:dt.sprite_frag},background:{uniforms:{uvTransform:{value:new ht},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:dt.background_vert,fragmentShader:dt.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new ht}},vertexShader:dt.backgroundCube_vert,fragmentShader:dt.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:dt.cube_vert,fragmentShader:dt.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:dt.equirect_vert,fragmentShader:dt.equirect_frag},distanceRGBA:{uniforms:Sn([Pe.common,Pe.displacementmap,{referencePosition:{value:new J},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:dt.distanceRGBA_vert,fragmentShader:dt.distanceRGBA_frag},shadow:{uniforms:Sn([Pe.lights,Pe.fog,{color:{value:new _t(0)},opacity:{value:1}}]),vertexShader:dt.shadow_vert,fragmentShader:dt.shadow_frag}};vi.physical={uniforms:Sn([vi.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ht},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ht},clearcoatNormalScale:{value:new ft(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ht},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ht},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ht},sheen:{value:0},sheenColor:{value:new _t(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ht},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ht},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ht},transmissionSamplerSize:{value:new ft},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ht},attenuationDistance:{value:0},attenuationColor:{value:new _t(0)},specularColor:{value:new _t(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ht},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ht},anisotropyVector:{value:new ft},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ht}}]),vertexShader:dt.meshphysical_vert,fragmentShader:dt.meshphysical_frag};const bl={r:0,b:0,g:0},Gr=new yi,nM=new Vt;function iM(s,e,t,r,o,u,c){const d=new _t(0);let h=u===!0?0:1,m,g,y=null,v=0,M=null;function T(P){let R=P.isScene===!0?P.background:null;return R&&R.isTexture&&(R=(P.backgroundBlurriness>0?t:e).get(R)),R}function S(P){let R=!1;const L=T(P);L===null?_(d,h):L&&L.isColor&&(_(L,1),R=!0);const $=s.xr.getEnvironmentBlendMode();$==="additive"?r.buffers.color.setClear(0,0,0,1,c):$==="alpha-blend"&&r.buffers.color.setClear(0,0,0,0,c),(s.autoClear||R)&&(r.buffers.depth.setTest(!0),r.buffers.depth.setMask(!0),r.buffers.color.setMask(!0),s.clear(s.autoClearColor,s.autoClearDepth,s.autoClearStencil))}function x(P,R){const L=T(R);L&&(L.isCubeTexture||L.mapping===tu)?(g===void 0&&(g=new zn(new ts(1,1,1),new Er({name:"BackgroundCubeMaterial",uniforms:sa(vi.backgroundCube.uniforms),vertexShader:vi.backgroundCube.vertexShader,fragmentShader:vi.backgroundCube.fragmentShader,side:Ln,depthTest:!1,depthWrite:!1,fog:!1})),g.geometry.deleteAttribute("normal"),g.geometry.deleteAttribute("uv"),g.onBeforeRender=function($,O,D){this.matrixWorld.copyPosition(D.matrixWorld)},Object.defineProperty(g.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),o.update(g)),Gr.copy(R.backgroundRotation),Gr.x*=-1,Gr.y*=-1,Gr.z*=-1,L.isCubeTexture&&L.isRenderTargetTexture===!1&&(Gr.y*=-1,Gr.z*=-1),g.material.uniforms.envMap.value=L,g.material.uniforms.flipEnvMap.value=L.isCubeTexture&&L.isRenderTargetTexture===!1?-1:1,g.material.uniforms.backgroundBlurriness.value=R.backgroundBlurriness,g.material.uniforms.backgroundIntensity.value=R.backgroundIntensity,g.material.uniforms.backgroundRotation.value.setFromMatrix4(nM.makeRotationFromEuler(Gr)),g.material.toneMapped=At.getTransfer(L.colorSpace)!==Ot,(y!==L||v!==L.version||M!==s.toneMapping)&&(g.material.needsUpdate=!0,y=L,v=L.version,M=s.toneMapping),g.layers.enableAll(),P.unshift(g,g.geometry,g.material,0,0,null)):L&&L.isTexture&&(m===void 0&&(m=new zn(new ru(2,2),new Er({name:"BackgroundMaterial",uniforms:sa(vi.background.uniforms),vertexShader:vi.background.vertexShader,fragmentShader:vi.background.fragmentShader,side:Mr,depthTest:!1,depthWrite:!1,fog:!1})),m.geometry.deleteAttribute("normal"),Object.defineProperty(m.material,"map",{get:function(){return this.uniforms.t2D.value}}),o.update(m)),m.material.uniforms.t2D.value=L,m.material.uniforms.backgroundIntensity.value=R.backgroundIntensity,m.material.toneMapped=At.getTransfer(L.colorSpace)!==Ot,L.matrixAutoUpdate===!0&&L.updateMatrix(),m.material.uniforms.uvTransform.value.copy(L.matrix),(y!==L||v!==L.version||M!==s.toneMapping)&&(m.material.needsUpdate=!0,y=L,v=L.version,M=s.toneMapping),m.layers.enableAll(),P.unshift(m,m.geometry,m.material,0,0,null))}function _(P,R){P.getRGB(bl,Yg(s)),r.buffers.color.setClear(bl.r,bl.g,bl.b,R,c)}return{getClearColor:function(){return d},setClearColor:function(P,R=1){d.set(P),h=R,_(d,h)},getClearAlpha:function(){return h},setClearAlpha:function(P){h=P,_(d,h)},render:S,addToRenderList:x}}function rM(s,e){const t=s.getParameter(s.MAX_VERTEX_ATTRIBS),r={},o=v(null);let u=o,c=!1;function d(w,I,Y,K,oe){let ne=!1;const B=y(K,Y,I);u!==B&&(u=B,m(u.object)),ne=M(w,K,Y,oe),ne&&T(w,K,Y,oe),oe!==null&&e.update(oe,s.ELEMENT_ARRAY_BUFFER),(ne||c)&&(c=!1,L(w,I,Y,K),oe!==null&&s.bindBuffer(s.ELEMENT_ARRAY_BUFFER,e.get(oe).buffer))}function h(){return s.createVertexArray()}function m(w){return s.bindVertexArray(w)}function g(w){return s.deleteVertexArray(w)}function y(w,I,Y){const K=Y.wireframe===!0;let oe=r[w.id];oe===void 0&&(oe={},r[w.id]=oe);let ne=oe[I.id];ne===void 0&&(ne={},oe[I.id]=ne);let B=ne[K];return B===void 0&&(B=v(h()),ne[K]=B),B}function v(w){const I=[],Y=[],K=[];for(let oe=0;oe<t;oe++)I[oe]=0,Y[oe]=0,K[oe]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:I,enabledAttributes:Y,attributeDivisors:K,object:w,attributes:{},index:null}}function M(w,I,Y,K){const oe=u.attributes,ne=I.attributes;let B=0;const G=Y.getAttributes();for(const k in G)if(G[k].location>=0){const le=oe[k];let F=ne[k];if(F===void 0&&(k==="instanceMatrix"&&w.instanceMatrix&&(F=w.instanceMatrix),k==="instanceColor"&&w.instanceColor&&(F=w.instanceColor)),le===void 0||le.attribute!==F||F&&le.data!==F.data)return!0;B++}return u.attributesNum!==B||u.index!==K}function T(w,I,Y,K){const oe={},ne=I.attributes;let B=0;const G=Y.getAttributes();for(const k in G)if(G[k].location>=0){let le=ne[k];le===void 0&&(k==="instanceMatrix"&&w.instanceMatrix&&(le=w.instanceMatrix),k==="instanceColor"&&w.instanceColor&&(le=w.instanceColor));const F={};F.attribute=le,le&&le.data&&(F.data=le.data),oe[k]=F,B++}u.attributes=oe,u.attributesNum=B,u.index=K}function S(){const w=u.newAttributes;for(let I=0,Y=w.length;I<Y;I++)w[I]=0}function x(w){_(w,0)}function _(w,I){const Y=u.newAttributes,K=u.enabledAttributes,oe=u.attributeDivisors;Y[w]=1,K[w]===0&&(s.enableVertexAttribArray(w),K[w]=1),oe[w]!==I&&(s.vertexAttribDivisor(w,I),oe[w]=I)}function P(){const w=u.newAttributes,I=u.enabledAttributes;for(let Y=0,K=I.length;Y<K;Y++)I[Y]!==w[Y]&&(s.disableVertexAttribArray(Y),I[Y]=0)}function R(w,I,Y,K,oe,ne,B){B===!0?s.vertexAttribIPointer(w,I,Y,oe,ne):s.vertexAttribPointer(w,I,Y,K,oe,ne)}function L(w,I,Y,K){S();const oe=K.attributes,ne=Y.getAttributes(),B=I.defaultAttributeValues;for(const G in ne){const k=ne[G];if(k.location>=0){let ue=oe[G];if(ue===void 0&&(G==="instanceMatrix"&&w.instanceMatrix&&(ue=w.instanceMatrix),G==="instanceColor"&&w.instanceColor&&(ue=w.instanceColor)),ue!==void 0){const le=ue.normalized,F=ue.itemSize,ce=e.get(ue);if(ce===void 0)continue;const Ie=ce.buffer,te=ce.type,fe=ce.bytesPerElement,xe=te===s.INT||te===s.UNSIGNED_INT||ue.gpuType===gd;if(ue.isInterleavedBufferAttribute){const Me=ue.data,Le=Me.stride,ke=ue.offset;if(Me.isInstancedInterleavedBuffer){for(let Ye=0;Ye<k.locationSize;Ye++)_(k.location+Ye,Me.meshPerAttribute);w.isInstancedMesh!==!0&&K._maxInstanceCount===void 0&&(K._maxInstanceCount=Me.meshPerAttribute*Me.count)}else for(let Ye=0;Ye<k.locationSize;Ye++)x(k.location+Ye);s.bindBuffer(s.ARRAY_BUFFER,Ie);for(let Ye=0;Ye<k.locationSize;Ye++)R(k.location+Ye,F/k.locationSize,te,le,Le*fe,(ke+F/k.locationSize*Ye)*fe,xe)}else{if(ue.isInstancedBufferAttribute){for(let Me=0;Me<k.locationSize;Me++)_(k.location+Me,ue.meshPerAttribute);w.isInstancedMesh!==!0&&K._maxInstanceCount===void 0&&(K._maxInstanceCount=ue.meshPerAttribute*ue.count)}else for(let Me=0;Me<k.locationSize;Me++)x(k.location+Me);s.bindBuffer(s.ARRAY_BUFFER,Ie);for(let Me=0;Me<k.locationSize;Me++)R(k.location+Me,F/k.locationSize,te,le,F*fe,F/k.locationSize*Me*fe,xe)}}else if(B!==void 0){const le=B[G];if(le!==void 0)switch(le.length){case 2:s.vertexAttrib2fv(k.location,le);break;case 3:s.vertexAttrib3fv(k.location,le);break;case 4:s.vertexAttrib4fv(k.location,le);break;default:s.vertexAttrib1fv(k.location,le)}}}}P()}function $(){j();for(const w in r){const I=r[w];for(const Y in I){const K=I[Y];for(const oe in K)g(K[oe].object),delete K[oe];delete I[Y]}delete r[w]}}function O(w){if(r[w.id]===void 0)return;const I=r[w.id];for(const Y in I){const K=I[Y];for(const oe in K)g(K[oe].object),delete K[oe];delete I[Y]}delete r[w.id]}function D(w){for(const I in r){const Y=r[I];if(Y[w.id]===void 0)continue;const K=Y[w.id];for(const oe in K)g(K[oe].object),delete K[oe];delete Y[w.id]}}function j(){b(),c=!0,u!==o&&(u=o,m(u.object))}function b(){o.geometry=null,o.program=null,o.wireframe=!1}return{setup:d,reset:j,resetDefaultState:b,dispose:$,releaseStatesOfGeometry:O,releaseStatesOfProgram:D,initAttributes:S,enableAttribute:x,disableUnusedAttributes:P}}function sM(s,e,t){let r;function o(m){r=m}function u(m,g){s.drawArrays(r,m,g),t.update(g,r,1)}function c(m,g,y){y!==0&&(s.drawArraysInstanced(r,m,g,y),t.update(g,r,y))}function d(m,g,y){if(y===0)return;e.get("WEBGL_multi_draw").multiDrawArraysWEBGL(r,m,0,g,0,y);let M=0;for(let T=0;T<y;T++)M+=g[T];t.update(M,r,1)}function h(m,g,y,v){if(y===0)return;const M=e.get("WEBGL_multi_draw");if(M===null)for(let T=0;T<m.length;T++)c(m[T],g[T],v[T]);else{M.multiDrawArraysInstancedWEBGL(r,m,0,g,0,v,0,y);let T=0;for(let S=0;S<y;S++)T+=g[S];for(let S=0;S<v.length;S++)t.update(T,r,v[S])}}this.setMode=o,this.render=u,this.renderInstances=c,this.renderMultiDraw=d,this.renderMultiDrawInstances=h}function aM(s,e,t,r){let o;function u(){if(o!==void 0)return o;if(e.has("EXT_texture_filter_anisotropic")===!0){const O=e.get("EXT_texture_filter_anisotropic");o=s.getParameter(O.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else o=0;return o}function c(O){return!(O!==fi&&r.convert(O)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_FORMAT))}function d(O){const D=O===eo&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(O!==Hi&&r.convert(O)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_TYPE)&&O!==Bi&&!D)}function h(O){if(O==="highp"){if(s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.HIGH_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.HIGH_FLOAT).precision>0)return"highp";O="mediump"}return O==="mediump"&&s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.MEDIUM_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let m=t.precision!==void 0?t.precision:"highp";const g=h(m);g!==m&&(console.warn("THREE.WebGLRenderer:",m,"not supported, using",g,"instead."),m=g);const y=t.logarithmicDepthBuffer===!0,v=s.getParameter(s.MAX_TEXTURE_IMAGE_UNITS),M=s.getParameter(s.MAX_VERTEX_TEXTURE_IMAGE_UNITS),T=s.getParameter(s.MAX_TEXTURE_SIZE),S=s.getParameter(s.MAX_CUBE_MAP_TEXTURE_SIZE),x=s.getParameter(s.MAX_VERTEX_ATTRIBS),_=s.getParameter(s.MAX_VERTEX_UNIFORM_VECTORS),P=s.getParameter(s.MAX_VARYING_VECTORS),R=s.getParameter(s.MAX_FRAGMENT_UNIFORM_VECTORS),L=M>0,$=s.getParameter(s.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:u,getMaxPrecision:h,textureFormatReadable:c,textureTypeReadable:d,precision:m,logarithmicDepthBuffer:y,maxTextures:v,maxVertexTextures:M,maxTextureSize:T,maxCubemapSize:S,maxAttributes:x,maxVertexUniforms:_,maxVaryings:P,maxFragmentUniforms:R,vertexTextures:L,maxSamples:$}}function oM(s){const e=this;let t=null,r=0,o=!1,u=!1;const c=new Xr,d=new ht,h={value:null,needsUpdate:!1};this.uniform=h,this.numPlanes=0,this.numIntersection=0,this.init=function(y,v){const M=y.length!==0||v||r!==0||o;return o=v,r=y.length,M},this.beginShadows=function(){u=!0,g(null)},this.endShadows=function(){u=!1},this.setGlobalState=function(y,v){t=g(y,v,0)},this.setState=function(y,v,M){const T=y.clippingPlanes,S=y.clipIntersection,x=y.clipShadows,_=s.get(y);if(!o||T===null||T.length===0||u&&!x)u?g(null):m();else{const P=u?0:r,R=P*4;let L=_.clippingState||null;h.value=L,L=g(T,v,R,M);for(let $=0;$!==R;++$)L[$]=t[$];_.clippingState=L,this.numIntersection=S?this.numPlanes:0,this.numPlanes+=P}};function m(){h.value!==t&&(h.value=t,h.needsUpdate=r>0),e.numPlanes=r,e.numIntersection=0}function g(y,v,M,T){const S=y!==null?y.length:0;let x=null;if(S!==0){if(x=h.value,T!==!0||x===null){const _=M+S*4,P=v.matrixWorldInverse;d.getNormalMatrix(P),(x===null||x.length<_)&&(x=new Float32Array(_));for(let R=0,L=M;R!==S;++R,L+=4)c.copy(y[R]).applyMatrix4(P,d),c.normal.toArray(x,L),x[L+3]=c.constant}h.value=x,h.needsUpdate=!0}return e.numPlanes=S,e.numIntersection=0,x}}function lM(s){let e=new WeakMap;function t(c,d){return d===Df?c.mapping=ta:d===If&&(c.mapping=na),c}function r(c){if(c&&c.isTexture){const d=c.mapping;if(d===Df||d===If)if(e.has(c)){const h=e.get(c).texture;return t(h,c.mapping)}else{const h=c.image;if(h&&h.height>0){const m=new xx(h.height);return m.fromEquirectangularTexture(s,c),e.set(c,m),c.addEventListener("dispose",o),t(m.texture,c.mapping)}else return null}}return c}function o(c){const d=c.target;d.removeEventListener("dispose",o);const h=e.get(d);h!==void 0&&(e.delete(d),h.dispose())}function u(){e=new WeakMap}return{get:r,dispose:u}}class qs extends qg{constructor(e=-1,t=1,r=1,o=-1,u=.1,c=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=r,this.bottom=o,this.near=u,this.far=c,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,r,o,u,c){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=r,this.view.offsetY=o,this.view.width=u,this.view.height=c,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),r=(this.right+this.left)/2,o=(this.top+this.bottom)/2;let u=r-e,c=r+e,d=o+t,h=o-t;if(this.view!==null&&this.view.enabled){const m=(this.right-this.left)/this.view.fullWidth/this.zoom,g=(this.top-this.bottom)/this.view.fullHeight/this.zoom;u+=m*this.view.offsetX,c=u+m*this.view.width,d-=g*this.view.offsetY,h=d-g*this.view.height}this.projectionMatrix.makeOrthographic(u,c,d,h,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}}const $s=4,Fm=[.125,.215,.35,.446,.526,.582],qr=20,hf=new qs,Om=new _t;let pf=null,mf=0,gf=0,_f=!1;const jr=(1+Math.sqrt(5))/2,Vs=1/jr,km=[new J(-jr,Vs,0),new J(jr,Vs,0),new J(-Vs,0,jr),new J(Vs,0,jr),new J(0,jr,-Vs),new J(0,jr,Vs),new J(-1,1,-1),new J(1,1,-1),new J(-1,1,1),new J(1,1,1)];class Bm{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,r=.1,o=100){pf=this._renderer.getRenderTarget(),mf=this._renderer.getActiveCubeFace(),gf=this._renderer.getActiveMipmapLevel(),_f=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);const u=this._allocateTargets();return u.depthBuffer=!0,this._sceneToCubeUV(e,r,o,u),t>0&&this._blur(u,0,0,t),this._applyPMREM(u),this._cleanup(u),u}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Vm(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Hm(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget(pf,mf,gf),this._renderer.xr.enabled=_f,e.scissorTest=!1,Pl(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===ta||e.mapping===na?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),pf=this._renderer.getRenderTarget(),mf=this._renderer.getActiveCubeFace(),gf=this._renderer.getActiveMipmapLevel(),_f=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const r=t||this._allocateTargets();return this._textureToCubeUV(e,r),this._applyPMREM(r),this._cleanup(r),r}_allocateTargets(){const e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,r={magFilter:ui,minFilter:ui,generateMipmaps:!1,type:eo,format:fi,colorSpace:wr,depthBuffer:!1},o=zm(e,t,r);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=zm(e,t,r);const{_lodMax:u}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=uM(u)),this._blurMaterial=cM(u,e,t)}return o}_compileMaterial(e){const t=new zn(this._lodPlanes[0],e);this._renderer.compile(t,hf)}_sceneToCubeUV(e,t,r,o){const d=new $n(90,1,t,r),h=[1,-1,1,1,1,1],m=[1,1,1,-1,-1,-1],g=this._renderer,y=g.autoClear,v=g.toneMapping;g.getClearColor(Om),g.toneMapping=yr,g.autoClear=!1;const M=new Ed({name:"PMREM.Background",side:Ln,depthWrite:!1,depthTest:!1}),T=new zn(new ts,M);let S=!1;const x=e.background;x?x.isColor&&(M.color.copy(x),e.background=null,S=!0):(M.color.copy(Om),S=!0);for(let _=0;_<6;_++){const P=_%3;P===0?(d.up.set(0,h[_],0),d.lookAt(m[_],0,0)):P===1?(d.up.set(0,0,h[_]),d.lookAt(0,m[_],0)):(d.up.set(0,h[_],0),d.lookAt(0,0,m[_]));const R=this._cubeSize;Pl(o,P*R,_>2?R:0,R,R),g.setRenderTarget(o),S&&g.render(T,d),g.render(e,d)}T.geometry.dispose(),T.material.dispose(),g.toneMapping=v,g.autoClear=y,e.background=x}_textureToCubeUV(e,t){const r=this._renderer,o=e.mapping===ta||e.mapping===na;o?(this._cubemapMaterial===null&&(this._cubemapMaterial=Vm()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Hm());const u=o?this._cubemapMaterial:this._equirectMaterial,c=new zn(this._lodPlanes[0],u),d=u.uniforms;d.envMap.value=e;const h=this._cubeSize;Pl(t,0,0,3*h,2*h),r.setRenderTarget(t),r.render(c,hf)}_applyPMREM(e){const t=this._renderer,r=t.autoClear;t.autoClear=!1;const o=this._lodPlanes.length;for(let u=1;u<o;u++){const c=Math.sqrt(this._sigmas[u]*this._sigmas[u]-this._sigmas[u-1]*this._sigmas[u-1]),d=km[(o-u-1)%km.length];this._blur(e,u-1,u,c,d)}t.autoClear=r}_blur(e,t,r,o,u){const c=this._pingPongRenderTarget;this._halfBlur(e,c,t,r,o,"latitudinal",u),this._halfBlur(c,e,r,r,o,"longitudinal",u)}_halfBlur(e,t,r,o,u,c,d){const h=this._renderer,m=this._blurMaterial;c!=="latitudinal"&&c!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const g=3,y=new zn(this._lodPlanes[o],m),v=m.uniforms,M=this._sizeLods[r]-1,T=isFinite(u)?Math.PI/(2*M):2*Math.PI/(2*qr-1),S=u/T,x=isFinite(u)?1+Math.floor(g*S):qr;x>qr&&console.warn(`sigmaRadians, ${u}, is too large and will clip, as it requested ${x} samples when the maximum is set to ${qr}`);const _=[];let P=0;for(let D=0;D<qr;++D){const j=D/S,b=Math.exp(-j*j/2);_.push(b),D===0?P+=b:D<x&&(P+=2*b)}for(let D=0;D<_.length;D++)_[D]=_[D]/P;v.envMap.value=e.texture,v.samples.value=x,v.weights.value=_,v.latitudinal.value=c==="latitudinal",d&&(v.poleAxis.value=d);const{_lodMax:R}=this;v.dTheta.value=T,v.mipInt.value=R-r;const L=this._sizeLods[o],$=3*L*(o>R-$s?o-R+$s:0),O=4*(this._cubeSize-L);Pl(t,$,O,3*L,2*L),h.setRenderTarget(t),h.render(y,hf)}}function uM(s){const e=[],t=[],r=[];let o=s;const u=s-$s+1+Fm.length;for(let c=0;c<u;c++){const d=Math.pow(2,o);t.push(d);let h=1/d;c>s-$s?h=Fm[c-s+$s-1]:c===0&&(h=0),r.push(h);const m=1/(d-2),g=-m,y=1+m,v=[g,g,y,g,y,y,g,g,y,y,g,y],M=6,T=6,S=3,x=2,_=1,P=new Float32Array(S*T*M),R=new Float32Array(x*T*M),L=new Float32Array(_*T*M);for(let O=0;O<M;O++){const D=O%3*2/3-1,j=O>2?0:-1,b=[D,j,0,D+2/3,j,0,D+2/3,j+1,0,D,j,0,D+2/3,j+1,0,D,j+1,0];P.set(b,S*T*O),R.set(v,x*T*O);const w=[O,O,O,O,O,O];L.set(w,_*T*O)}const $=new Hn;$.setAttribute("position",new di(P,S)),$.setAttribute("uv",new di(R,x)),$.setAttribute("faceIndex",new di(L,_)),e.push($),o>$s&&o--}return{lodPlanes:e,sizeLods:t,sigmas:r}}function zm(s,e,t){const r=new es(s,e,t);return r.texture.mapping=tu,r.texture.name="PMREM.cubeUv",r.scissorTest=!0,r}function Pl(s,e,t,r,o){s.viewport.set(e,t,r,o),s.scissor.set(e,t,r,o)}function cM(s,e,t){const r=new Float32Array(qr),o=new J(0,1,0);return new Er({name:"SphericalGaussianBlur",defines:{n:qr,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${s}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:r},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:o}},vertexShader:Td(),fragmentShader:`

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
		`,blending:xr,depthTest:!1,depthWrite:!1})}function Hm(){return new Er({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Td(),fragmentShader:`

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
		`,blending:xr,depthTest:!1,depthWrite:!1})}function Vm(){return new Er({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Td(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:xr,depthTest:!1,depthWrite:!1})}function Td(){return`

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
	`}function fM(s){let e=new WeakMap,t=null;function r(d){if(d&&d.isTexture){const h=d.mapping,m=h===Df||h===If,g=h===ta||h===na;if(m||g){let y=e.get(d);const v=y!==void 0?y.texture.pmremVersion:0;if(d.isRenderTargetTexture&&d.pmremVersion!==v)return t===null&&(t=new Bm(s)),y=m?t.fromEquirectangular(d,y):t.fromCubemap(d,y),y.texture.pmremVersion=d.pmremVersion,e.set(d,y),y.texture;if(y!==void 0)return y.texture;{const M=d.image;return m&&M&&M.height>0||g&&M&&o(M)?(t===null&&(t=new Bm(s)),y=m?t.fromEquirectangular(d):t.fromCubemap(d),y.texture.pmremVersion=d.pmremVersion,e.set(d,y),d.addEventListener("dispose",u),y.texture):null}}}return d}function o(d){let h=0;const m=6;for(let g=0;g<m;g++)d[g]!==void 0&&h++;return h===m}function u(d){const h=d.target;h.removeEventListener("dispose",u);const m=e.get(h);m!==void 0&&(e.delete(h),m.dispose())}function c(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:r,dispose:c}}function dM(s){const e={};function t(r){if(e[r]!==void 0)return e[r];let o;switch(r){case"WEBGL_depth_texture":o=s.getExtension("WEBGL_depth_texture")||s.getExtension("MOZ_WEBGL_depth_texture")||s.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":o=s.getExtension("EXT_texture_filter_anisotropic")||s.getExtension("MOZ_EXT_texture_filter_anisotropic")||s.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":o=s.getExtension("WEBGL_compressed_texture_s3tc")||s.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":o=s.getExtension("WEBGL_compressed_texture_pvrtc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:o=s.getExtension(r)}return e[r]=o,o}return{has:function(r){return t(r)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(r){const o=t(r);return o===null&&Js("THREE.WebGLRenderer: "+r+" extension not supported."),o}}}function hM(s,e,t,r){const o={},u=new WeakMap;function c(y){const v=y.target;v.index!==null&&e.remove(v.index);for(const T in v.attributes)e.remove(v.attributes[T]);for(const T in v.morphAttributes){const S=v.morphAttributes[T];for(let x=0,_=S.length;x<_;x++)e.remove(S[x])}v.removeEventListener("dispose",c),delete o[v.id];const M=u.get(v);M&&(e.remove(M),u.delete(v)),r.releaseStatesOfGeometry(v),v.isInstancedBufferGeometry===!0&&delete v._maxInstanceCount,t.memory.geometries--}function d(y,v){return o[v.id]===!0||(v.addEventListener("dispose",c),o[v.id]=!0,t.memory.geometries++),v}function h(y){const v=y.attributes;for(const T in v)e.update(v[T],s.ARRAY_BUFFER);const M=y.morphAttributes;for(const T in M){const S=M[T];for(let x=0,_=S.length;x<_;x++)e.update(S[x],s.ARRAY_BUFFER)}}function m(y){const v=[],M=y.index,T=y.attributes.position;let S=0;if(M!==null){const P=M.array;S=M.version;for(let R=0,L=P.length;R<L;R+=3){const $=P[R+0],O=P[R+1],D=P[R+2];v.push($,O,O,D,D,$)}}else if(T!==void 0){const P=T.array;S=T.version;for(let R=0,L=P.length/3-1;R<L;R+=3){const $=R+0,O=R+1,D=R+2;v.push($,O,O,D,D,$)}}else return;const x=new(Bg(v)?jg:Xg)(v,1);x.version=S;const _=u.get(y);_&&e.remove(_),u.set(y,x)}function g(y){const v=u.get(y);if(v){const M=y.index;M!==null&&v.version<M.version&&m(y)}else m(y);return u.get(y)}return{get:d,update:h,getWireframeAttribute:g}}function pM(s,e,t){let r;function o(v){r=v}let u,c;function d(v){u=v.type,c=v.bytesPerElement}function h(v,M){s.drawElements(r,M,u,v*c),t.update(M,r,1)}function m(v,M,T){T!==0&&(s.drawElementsInstanced(r,M,u,v*c,T),t.update(M,r,T))}function g(v,M,T){if(T===0)return;e.get("WEBGL_multi_draw").multiDrawElementsWEBGL(r,M,0,u,v,0,T);let x=0;for(let _=0;_<T;_++)x+=M[_];t.update(x,r,1)}function y(v,M,T,S){if(T===0)return;const x=e.get("WEBGL_multi_draw");if(x===null)for(let _=0;_<v.length;_++)m(v[_]/c,M[_],S[_]);else{x.multiDrawElementsInstancedWEBGL(r,M,0,u,v,0,S,0,T);let _=0;for(let P=0;P<T;P++)_+=M[P];for(let P=0;P<S.length;P++)t.update(_,r,S[P])}}this.setMode=o,this.setIndex=d,this.render=h,this.renderInstances=m,this.renderMultiDraw=g,this.renderMultiDrawInstances=y}function mM(s){const e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function r(u,c,d){switch(t.calls++,c){case s.TRIANGLES:t.triangles+=d*(u/3);break;case s.LINES:t.lines+=d*(u/2);break;case s.LINE_STRIP:t.lines+=d*(u-1);break;case s.LINE_LOOP:t.lines+=d*u;break;case s.POINTS:t.points+=d*u;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",c);break}}function o(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:o,update:r}}function gM(s,e,t){const r=new WeakMap,o=new Zt;function u(c,d,h){const m=c.morphTargetInfluences,g=d.morphAttributes.position||d.morphAttributes.normal||d.morphAttributes.color,y=g!==void 0?g.length:0;let v=r.get(d);if(v===void 0||v.count!==y){let w=function(){j.dispose(),r.delete(d),d.removeEventListener("dispose",w)};var M=w;v!==void 0&&v.texture.dispose();const T=d.morphAttributes.position!==void 0,S=d.morphAttributes.normal!==void 0,x=d.morphAttributes.color!==void 0,_=d.morphAttributes.position||[],P=d.morphAttributes.normal||[],R=d.morphAttributes.color||[];let L=0;T===!0&&(L=1),S===!0&&(L=2),x===!0&&(L=3);let $=d.attributes.position.count*L,O=1;$>e.maxTextureSize&&(O=Math.ceil($/e.maxTextureSize),$=e.maxTextureSize);const D=new Float32Array($*O*4*y),j=new Hg(D,$,O,y);j.type=Bi,j.needsUpdate=!0;const b=L*4;for(let I=0;I<y;I++){const Y=_[I],K=P[I],oe=R[I],ne=$*O*4*I;for(let B=0;B<Y.count;B++){const G=B*b;T===!0&&(o.fromBufferAttribute(Y,B),D[ne+G+0]=o.x,D[ne+G+1]=o.y,D[ne+G+2]=o.z,D[ne+G+3]=0),S===!0&&(o.fromBufferAttribute(K,B),D[ne+G+4]=o.x,D[ne+G+5]=o.y,D[ne+G+6]=o.z,D[ne+G+7]=0),x===!0&&(o.fromBufferAttribute(oe,B),D[ne+G+8]=o.x,D[ne+G+9]=o.y,D[ne+G+10]=o.z,D[ne+G+11]=oe.itemSize===4?o.w:1)}}v={count:y,texture:j,size:new ft($,O)},r.set(d,v),d.addEventListener("dispose",w)}if(c.isInstancedMesh===!0&&c.morphTexture!==null)h.getUniforms().setValue(s,"morphTexture",c.morphTexture,t);else{let T=0;for(let x=0;x<m.length;x++)T+=m[x];const S=d.morphTargetsRelative?1:1-T;h.getUniforms().setValue(s,"morphTargetBaseInfluence",S),h.getUniforms().setValue(s,"morphTargetInfluences",m)}h.getUniforms().setValue(s,"morphTargetsTexture",v.texture,t),h.getUniforms().setValue(s,"morphTargetsTextureSize",v.size)}return{update:u}}function _M(s,e,t,r){let o=new WeakMap;function u(h){const m=r.render.frame,g=h.geometry,y=e.get(h,g);if(o.get(y)!==m&&(e.update(y),o.set(y,m)),h.isInstancedMesh&&(h.hasEventListener("dispose",d)===!1&&h.addEventListener("dispose",d),o.get(h)!==m&&(t.update(h.instanceMatrix,s.ARRAY_BUFFER),h.instanceColor!==null&&t.update(h.instanceColor,s.ARRAY_BUFFER),o.set(h,m))),h.isSkinnedMesh){const v=h.skeleton;o.get(v)!==m&&(v.update(),o.set(v,m))}return y}function c(){o=new WeakMap}function d(h){const m=h.target;m.removeEventListener("dispose",d),t.remove(m.instanceMatrix),m.instanceColor!==null&&t.remove(m.instanceColor)}return{update:u,dispose:c}}class Zg extends En{constructor(e,t,r,o,u,c,d,h,m,g=Qs){if(g!==Qs&&g!==ra)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");r===void 0&&g===Qs&&(r=Jr),r===void 0&&g===ra&&(r=ia),super(null,o,u,c,d,h,g,r,m),this.isDepthTexture=!0,this.image={width:e,height:t},this.magFilter=d!==void 0?d:Kn,this.minFilter=h!==void 0?h:Kn,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.compareFunction=e.compareFunction,this}toJSON(e){const t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}}const Qg=new En,Gm=new Zg(1,1),Jg=new Hg,e_=new ix,t_=new $g,Wm=[],Xm=[],jm=new Float32Array(16),Ym=new Float32Array(9),qm=new Float32Array(4);function oa(s,e,t){const r=s[0];if(r<=0||r>0)return s;const o=e*t;let u=Wm[o];if(u===void 0&&(u=new Float32Array(o),Wm[o]=u),e!==0){r.toArray(u,0);for(let c=1,d=0;c!==e;++c)d+=t,s[c].toArray(u,d)}return u}function Jt(s,e){if(s.length!==e.length)return!1;for(let t=0,r=s.length;t<r;t++)if(s[t]!==e[t])return!1;return!0}function en(s,e){for(let t=0,r=e.length;t<r;t++)s[t]=e[t]}function su(s,e){let t=Xm[e];t===void 0&&(t=new Int32Array(e),Xm[e]=t);for(let r=0;r!==e;++r)t[r]=s.allocateTextureUnit();return t}function vM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1f(this.addr,e),t[0]=e)}function xM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2fv(this.addr,e),en(t,e)}}function yM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(s.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(Jt(t,e))return;s.uniform3fv(this.addr,e),en(t,e)}}function SM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4fv(this.addr,e),en(t,e)}}function MM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix2fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;qm.set(r),s.uniformMatrix2fv(this.addr,!1,qm),en(t,r)}}function EM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix3fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;Ym.set(r),s.uniformMatrix3fv(this.addr,!1,Ym),en(t,r)}}function wM(s,e){const t=this.cache,r=e.elements;if(r===void 0){if(Jt(t,e))return;s.uniformMatrix4fv(this.addr,!1,e),en(t,e)}else{if(Jt(t,r))return;jm.set(r),s.uniformMatrix4fv(this.addr,!1,jm),en(t,r)}}function TM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1i(this.addr,e),t[0]=e)}function AM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2iv(this.addr,e),en(t,e)}}function CM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Jt(t,e))return;s.uniform3iv(this.addr,e),en(t,e)}}function RM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4iv(this.addr,e),en(t,e)}}function bM(s,e){const t=this.cache;t[0]!==e&&(s.uniform1ui(this.addr,e),t[0]=e)}function PM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Jt(t,e))return;s.uniform2uiv(this.addr,e),en(t,e)}}function LM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Jt(t,e))return;s.uniform3uiv(this.addr,e),en(t,e)}}function NM(s,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Jt(t,e))return;s.uniform4uiv(this.addr,e),en(t,e)}}function DM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o);let u;this.type===s.SAMPLER_2D_SHADOW?(Gm.compareFunction=kg,u=Gm):u=Qg,t.setTexture2D(e||u,o)}function IM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTexture3D(e||e_,o)}function UM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTextureCube(e||t_,o)}function FM(s,e,t){const r=this.cache,o=t.allocateTextureUnit();r[0]!==o&&(s.uniform1i(this.addr,o),r[0]=o),t.setTexture2DArray(e||Jg,o)}function OM(s){switch(s){case 5126:return vM;case 35664:return xM;case 35665:return yM;case 35666:return SM;case 35674:return MM;case 35675:return EM;case 35676:return wM;case 5124:case 35670:return TM;case 35667:case 35671:return AM;case 35668:case 35672:return CM;case 35669:case 35673:return RM;case 5125:return bM;case 36294:return PM;case 36295:return LM;case 36296:return NM;case 35678:case 36198:case 36298:case 36306:case 35682:return DM;case 35679:case 36299:case 36307:return IM;case 35680:case 36300:case 36308:case 36293:return UM;case 36289:case 36303:case 36311:case 36292:return FM}}function kM(s,e){s.uniform1fv(this.addr,e)}function BM(s,e){const t=oa(e,this.size,2);s.uniform2fv(this.addr,t)}function zM(s,e){const t=oa(e,this.size,3);s.uniform3fv(this.addr,t)}function HM(s,e){const t=oa(e,this.size,4);s.uniform4fv(this.addr,t)}function VM(s,e){const t=oa(e,this.size,4);s.uniformMatrix2fv(this.addr,!1,t)}function GM(s,e){const t=oa(e,this.size,9);s.uniformMatrix3fv(this.addr,!1,t)}function WM(s,e){const t=oa(e,this.size,16);s.uniformMatrix4fv(this.addr,!1,t)}function XM(s,e){s.uniform1iv(this.addr,e)}function jM(s,e){s.uniform2iv(this.addr,e)}function YM(s,e){s.uniform3iv(this.addr,e)}function qM(s,e){s.uniform4iv(this.addr,e)}function $M(s,e){s.uniform1uiv(this.addr,e)}function KM(s,e){s.uniform2uiv(this.addr,e)}function ZM(s,e){s.uniform3uiv(this.addr,e)}function QM(s,e){s.uniform4uiv(this.addr,e)}function JM(s,e,t){const r=this.cache,o=e.length,u=su(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture2D(e[c]||Qg,u[c])}function eE(s,e,t){const r=this.cache,o=e.length,u=su(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture3D(e[c]||e_,u[c])}function tE(s,e,t){const r=this.cache,o=e.length,u=su(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTextureCube(e[c]||t_,u[c])}function nE(s,e,t){const r=this.cache,o=e.length,u=su(t,o);Jt(r,u)||(s.uniform1iv(this.addr,u),en(r,u));for(let c=0;c!==o;++c)t.setTexture2DArray(e[c]||Jg,u[c])}function iE(s){switch(s){case 5126:return kM;case 35664:return BM;case 35665:return zM;case 35666:return HM;case 35674:return VM;case 35675:return GM;case 35676:return WM;case 5124:case 35670:return XM;case 35667:case 35671:return jM;case 35668:case 35672:return YM;case 35669:case 35673:return qM;case 5125:return $M;case 36294:return KM;case 36295:return ZM;case 36296:return QM;case 35678:case 36198:case 36298:case 36306:case 35682:return JM;case 35679:case 36299:case 36307:return eE;case 35680:case 36300:case 36308:case 36293:return tE;case 36289:case 36303:case 36311:case 36292:return nE}}class rE{constructor(e,t,r){this.id=e,this.addr=r,this.cache=[],this.type=t.type,this.setValue=OM(t.type)}}class sE{constructor(e,t,r){this.id=e,this.addr=r,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=iE(t.type)}}class aE{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,r){const o=this.seq;for(let u=0,c=o.length;u!==c;++u){const d=o[u];d.setValue(e,t[d.id],r)}}}const vf=/(\w+)(\])?(\[|\.)?/g;function $m(s,e){s.seq.push(e),s.map[e.id]=e}function oE(s,e,t){const r=s.name,o=r.length;for(vf.lastIndex=0;;){const u=vf.exec(r),c=vf.lastIndex;let d=u[1];const h=u[2]==="]",m=u[3];if(h&&(d=d|0),m===void 0||m==="["&&c+2===o){$m(t,m===void 0?new rE(d,s,e):new sE(d,s,e));break}else{let y=t.map[d];y===void 0&&(y=new aE(d),$m(t,y)),t=y}}}class Xl{constructor(e,t){this.seq=[],this.map={};const r=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let o=0;o<r;++o){const u=e.getActiveUniform(t,o),c=e.getUniformLocation(t,u.name);oE(u,c,this)}}setValue(e,t,r,o){const u=this.map[t];u!==void 0&&u.setValue(e,r,o)}setOptional(e,t,r){const o=t[r];o!==void 0&&this.setValue(e,r,o)}static upload(e,t,r,o){for(let u=0,c=t.length;u!==c;++u){const d=t[u],h=r[d.id];h.needsUpdate!==!1&&d.setValue(e,h.value,o)}}static seqWithValue(e,t){const r=[];for(let o=0,u=e.length;o!==u;++o){const c=e[o];c.id in t&&r.push(c)}return r}}function Km(s,e,t){const r=s.createShader(e);return s.shaderSource(r,t),s.compileShader(r),r}const lE=37297;let uE=0;function cE(s,e){const t=s.split(`
`),r=[],o=Math.max(e-6,0),u=Math.min(e+6,t.length);for(let c=o;c<u;c++){const d=c+1;r.push(`${d===e?">":" "} ${d}: ${t[c]}`)}return r.join(`
`)}function fE(s){const e=At.getPrimaries(At.workingColorSpace),t=At.getPrimaries(s);let r;switch(e===t?r="":e===$l&&t===ql?r="LinearDisplayP3ToLinearSRGB":e===ql&&t===$l&&(r="LinearSRGBToLinearDisplayP3"),s){case wr:case nu:return[r,"LinearTransferOETF"];case li:case Md:return[r,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",s),[r,"LinearTransferOETF"]}}function Zm(s,e,t){const r=s.getShaderParameter(e,s.COMPILE_STATUS),o=s.getShaderInfoLog(e).trim();if(r&&o==="")return"";const u=/ERROR: 0:(\d+)/.exec(o);if(u){const c=parseInt(u[1]);return t.toUpperCase()+`

`+o+`

`+cE(s.getShaderSource(e),c)}else return o}function dE(s,e){const t=fE(e);return`vec4 ${s}( vec4 value ) { return ${t[0]}( ${t[1]}( value ) ); }`}function hE(s,e){let t;switch(e){case L0:t="Linear";break;case N0:t="Reinhard";break;case D0:t="OptimizedCineon";break;case I0:t="ACESFilmic";break;case F0:t="AgX";break;case O0:t="Neutral";break;case U0:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+s+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}const Ll=new J;function pE(){At.getLuminanceCoefficients(Ll);const s=Ll.x.toFixed(4),e=Ll.y.toFixed(4),t=Ll.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${s}, ${e}, ${t} );`,"	return dot( weights, rgb );","}"].join(`
`)}function mE(s){return[s.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",s.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(Za).join(`
`)}function gE(s){const e=[];for(const t in s){const r=s[t];r!==!1&&e.push("#define "+t+" "+r)}return e.join(`
`)}function _E(s,e){const t={},r=s.getProgramParameter(e,s.ACTIVE_ATTRIBUTES);for(let o=0;o<r;o++){const u=s.getActiveAttrib(e,o),c=u.name;let d=1;u.type===s.FLOAT_MAT2&&(d=2),u.type===s.FLOAT_MAT3&&(d=3),u.type===s.FLOAT_MAT4&&(d=4),t[c]={type:u.type,location:s.getAttribLocation(e,c),locationSize:d}}return t}function Za(s){return s!==""}function Qm(s,e){const t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return s.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function Jm(s,e){return s.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}const vE=/^[ \t]*#include +<([\w\d./]+)>/gm;function fd(s){return s.replace(vE,yE)}const xE=new Map;function yE(s,e){let t=dt[e];if(t===void 0){const r=xE.get(e);if(r!==void 0)t=dt[r],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,r);else throw new Error("Can not resolve #include <"+e+">")}return fd(t)}const SE=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function eg(s){return s.replace(SE,ME)}function ME(s,e,t,r){let o="";for(let u=parseInt(e);u<parseInt(t);u++)o+=r.replace(/\[\s*i\s*\]/g,"[ "+u+" ]").replace(/UNROLLED_LOOP_INDEX/g,u);return o}function tg(s){let e=`precision ${s.precision} float;
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
#define LOW_PRECISION`),e}function EE(s){let e="SHADOWMAP_TYPE_BASIC";return s.shadowMapType===wg?e="SHADOWMAP_TYPE_PCF":s.shadowMapType===i0?e="SHADOWMAP_TYPE_PCF_SOFT":s.shadowMapType===Oi&&(e="SHADOWMAP_TYPE_VSM"),e}function wE(s){let e="ENVMAP_TYPE_CUBE";if(s.envMap)switch(s.envMapMode){case ta:case na:e="ENVMAP_TYPE_CUBE";break;case tu:e="ENVMAP_TYPE_CUBE_UV";break}return e}function TE(s){let e="ENVMAP_MODE_REFLECTION";if(s.envMap)switch(s.envMapMode){case na:e="ENVMAP_MODE_REFRACTION";break}return e}function AE(s){let e="ENVMAP_BLENDING_NONE";if(s.envMap)switch(s.combine){case Tg:e="ENVMAP_BLENDING_MULTIPLY";break;case b0:e="ENVMAP_BLENDING_MIX";break;case P0:e="ENVMAP_BLENDING_ADD";break}return e}function CE(s){const e=s.envMapCubeUVHeight;if(e===null)return null;const t=Math.log2(e)-2,r=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),112)),texelHeight:r,maxMip:t}}function RE(s,e,t,r){const o=s.getContext(),u=t.defines;let c=t.vertexShader,d=t.fragmentShader;const h=EE(t),m=wE(t),g=TE(t),y=AE(t),v=CE(t),M=mE(t),T=gE(u),S=o.createProgram();let x,_,P=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(x=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T].filter(Za).join(`
`),x.length>0&&(x+=`
`),_=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T].filter(Za).join(`
`),_.length>0&&(_+=`
`)):(x=[tg(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+g:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+h:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(Za).join(`
`),_=[tg(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,T,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+m:"",t.envMap?"#define "+g:"",t.envMap?"#define "+y:"",v?"#define CUBEUV_TEXEL_WIDTH "+v.texelWidth:"",v?"#define CUBEUV_TEXEL_HEIGHT "+v.texelHeight:"",v?"#define CUBEUV_MAX_MIP "+v.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+h:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==yr?"#define TONE_MAPPING":"",t.toneMapping!==yr?dt.tonemapping_pars_fragment:"",t.toneMapping!==yr?hE("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",dt.colorspace_pars_fragment,dE("linearToOutputTexel",t.outputColorSpace),pE(),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(Za).join(`
`)),c=fd(c),c=Qm(c,t),c=Jm(c,t),d=fd(d),d=Qm(d,t),d=Jm(d,t),c=eg(c),d=eg(d),t.isRawShaderMaterial!==!0&&(P=`#version 300 es
`,x=[M,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+x,_=["#define varying in",t.glslVersion===gm?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===gm?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+_);const R=P+x+c,L=P+_+d,$=Km(o,o.VERTEX_SHADER,R),O=Km(o,o.FRAGMENT_SHADER,L);o.attachShader(S,$),o.attachShader(S,O),t.index0AttributeName!==void 0?o.bindAttribLocation(S,0,t.index0AttributeName):t.morphTargets===!0&&o.bindAttribLocation(S,0,"position"),o.linkProgram(S);function D(I){if(s.debug.checkShaderErrors){const Y=o.getProgramInfoLog(S).trim(),K=o.getShaderInfoLog($).trim(),oe=o.getShaderInfoLog(O).trim();let ne=!0,B=!0;if(o.getProgramParameter(S,o.LINK_STATUS)===!1)if(ne=!1,typeof s.debug.onShaderError=="function")s.debug.onShaderError(o,S,$,O);else{const G=Zm(o,$,"vertex"),k=Zm(o,O,"fragment");console.error("THREE.WebGLProgram: Shader Error "+o.getError()+" - VALIDATE_STATUS "+o.getProgramParameter(S,o.VALIDATE_STATUS)+`

Material Name: `+I.name+`
Material Type: `+I.type+`

Program Info Log: `+Y+`
`+G+`
`+k)}else Y!==""?console.warn("THREE.WebGLProgram: Program Info Log:",Y):(K===""||oe==="")&&(B=!1);B&&(I.diagnostics={runnable:ne,programLog:Y,vertexShader:{log:K,prefix:x},fragmentShader:{log:oe,prefix:_}})}o.deleteShader($),o.deleteShader(O),j=new Xl(o,S),b=_E(o,S)}let j;this.getUniforms=function(){return j===void 0&&D(this),j};let b;this.getAttributes=function(){return b===void 0&&D(this),b};let w=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return w===!1&&(w=o.getProgramParameter(S,lE)),w},this.destroy=function(){r.releaseStatesOfProgram(this),o.deleteProgram(S),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=uE++,this.cacheKey=e,this.usedTimes=1,this.program=S,this.vertexShader=$,this.fragmentShader=O,this}let bE=0;class PE{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){const t=e.vertexShader,r=e.fragmentShader,o=this._getShaderStage(t),u=this._getShaderStage(r),c=this._getShaderCacheForMaterial(e);return c.has(o)===!1&&(c.add(o),o.usedTimes++),c.has(u)===!1&&(c.add(u),u.usedTimes++),this}remove(e){const t=this.materialCache.get(e);for(const r of t)r.usedTimes--,r.usedTimes===0&&this.shaderCache.delete(r.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){const t=this.materialCache;let r=t.get(e);return r===void 0&&(r=new Set,t.set(e,r)),r}_getShaderStage(e){const t=this.shaderCache;let r=t.get(e);return r===void 0&&(r=new LE(e),t.set(e,r)),r}}class LE{constructor(e){this.id=bE++,this.code=e,this.usedTimes=0}}function NE(s,e,t,r,o,u,c){const d=new Gg,h=new PE,m=new Set,g=[],y=o.logarithmicDepthBuffer,v=o.vertexTextures;let M=o.precision;const T={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function S(b){return m.add(b),b===0?"uv":`uv${b}`}function x(b,w,I,Y,K){const oe=Y.fog,ne=K.geometry,B=b.isMeshStandardMaterial?Y.environment:null,G=(b.isMeshStandardMaterial?t:e).get(b.envMap||B),k=G&&G.mapping===tu?G.image.height:null,ue=T[b.type];b.precision!==null&&(M=o.getMaxPrecision(b.precision),M!==b.precision&&console.warn("THREE.WebGLProgram.getParameters:",b.precision,"not supported, using",M,"instead."));const le=ne.morphAttributes.position||ne.morphAttributes.normal||ne.morphAttributes.color,F=le!==void 0?le.length:0;let ce=0;ne.morphAttributes.position!==void 0&&(ce=1),ne.morphAttributes.normal!==void 0&&(ce=2),ne.morphAttributes.color!==void 0&&(ce=3);let Ie,te,fe,xe;if(ue){const ut=vi[ue];Ie=ut.vertexShader,te=ut.fragmentShader}else Ie=b.vertexShader,te=b.fragmentShader,h.update(b),fe=h.getVertexShaderID(b),xe=h.getFragmentShaderID(b);const Me=s.getRenderTarget(),Le=K.isInstancedMesh===!0,ke=K.isBatchedMesh===!0,Ye=!!b.map,wt=!!b.matcap,z=!!G,bt=!!b.aoMap,vt=!!b.lightMap,yt=!!b.bumpMap,We=!!b.normalMap,Lt=!!b.displacementMap,tt=!!b.emissiveMap,rt=!!b.metalnessMap,U=!!b.roughnessMap,A=b.anisotropy>0,se=b.clearcoat>0,_e=b.dispersion>0,ye=b.iridescence>0,me=b.sheen>0,je=b.transmission>0,be=A&&!!b.anisotropyMap,De=se&&!!b.clearcoatMap,ot=se&&!!b.clearcoatNormalMap,Ee=se&&!!b.clearcoatRoughnessMap,Ne=ye&&!!b.iridescenceMap,pt=ye&&!!b.iridescenceThicknessMap,Je=me&&!!b.sheenColorMap,Oe=me&&!!b.sheenRoughnessMap,st=!!b.specularMap,lt=!!b.specularColorMap,Tt=!!b.specularIntensityMap,X=je&&!!b.transmissionMap,we=je&&!!b.thicknessMap,de=!!b.gradientMap,he=!!b.alphaMap,Te=b.alphaTest>0,qe=!!b.alphaHash,pe=!!b.extensions;let Ke=yr;b.toneMapped&&(Me===null||Me.isXRRenderTarget===!0)&&(Ke=s.toneMapping);const mt={shaderID:ue,shaderType:b.type,shaderName:b.name,vertexShader:Ie,fragmentShader:te,defines:b.defines,customVertexShaderID:fe,customFragmentShaderID:xe,isRawShaderMaterial:b.isRawShaderMaterial===!0,glslVersion:b.glslVersion,precision:M,batching:ke,batchingColor:ke&&K._colorsTexture!==null,instancing:Le,instancingColor:Le&&K.instanceColor!==null,instancingMorph:Le&&K.morphTexture!==null,supportsVertexTextures:v,outputColorSpace:Me===null?s.outputColorSpace:Me.isXRRenderTarget===!0?Me.texture.colorSpace:wr,alphaToCoverage:!!b.alphaToCoverage,map:Ye,matcap:wt,envMap:z,envMapMode:z&&G.mapping,envMapCubeUVHeight:k,aoMap:bt,lightMap:vt,bumpMap:yt,normalMap:We,displacementMap:v&&Lt,emissiveMap:tt,normalMapObjectSpace:We&&b.normalMapType===H0,normalMapTangentSpace:We&&b.normalMapType===Og,metalnessMap:rt,roughnessMap:U,anisotropy:A,anisotropyMap:be,clearcoat:se,clearcoatMap:De,clearcoatNormalMap:ot,clearcoatRoughnessMap:Ee,dispersion:_e,iridescence:ye,iridescenceMap:Ne,iridescenceThicknessMap:pt,sheen:me,sheenColorMap:Je,sheenRoughnessMap:Oe,specularMap:st,specularColorMap:lt,specularIntensityMap:Tt,transmission:je,transmissionMap:X,thicknessMap:we,gradientMap:de,opaque:b.transparent===!1&&b.blending===Zs&&b.alphaToCoverage===!1,alphaMap:he,alphaTest:Te,alphaHash:qe,combine:b.combine,mapUv:Ye&&S(b.map.channel),aoMapUv:bt&&S(b.aoMap.channel),lightMapUv:vt&&S(b.lightMap.channel),bumpMapUv:yt&&S(b.bumpMap.channel),normalMapUv:We&&S(b.normalMap.channel),displacementMapUv:Lt&&S(b.displacementMap.channel),emissiveMapUv:tt&&S(b.emissiveMap.channel),metalnessMapUv:rt&&S(b.metalnessMap.channel),roughnessMapUv:U&&S(b.roughnessMap.channel),anisotropyMapUv:be&&S(b.anisotropyMap.channel),clearcoatMapUv:De&&S(b.clearcoatMap.channel),clearcoatNormalMapUv:ot&&S(b.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Ee&&S(b.clearcoatRoughnessMap.channel),iridescenceMapUv:Ne&&S(b.iridescenceMap.channel),iridescenceThicknessMapUv:pt&&S(b.iridescenceThicknessMap.channel),sheenColorMapUv:Je&&S(b.sheenColorMap.channel),sheenRoughnessMapUv:Oe&&S(b.sheenRoughnessMap.channel),specularMapUv:st&&S(b.specularMap.channel),specularColorMapUv:lt&&S(b.specularColorMap.channel),specularIntensityMapUv:Tt&&S(b.specularIntensityMap.channel),transmissionMapUv:X&&S(b.transmissionMap.channel),thicknessMapUv:we&&S(b.thicknessMap.channel),alphaMapUv:he&&S(b.alphaMap.channel),vertexTangents:!!ne.attributes.tangent&&(We||A),vertexColors:b.vertexColors,vertexAlphas:b.vertexColors===!0&&!!ne.attributes.color&&ne.attributes.color.itemSize===4,pointsUvs:K.isPoints===!0&&!!ne.attributes.uv&&(Ye||he),fog:!!oe,useFog:b.fog===!0,fogExp2:!!oe&&oe.isFogExp2,flatShading:b.flatShading===!0,sizeAttenuation:b.sizeAttenuation===!0,logarithmicDepthBuffer:y,skinning:K.isSkinnedMesh===!0,morphTargets:ne.morphAttributes.position!==void 0,morphNormals:ne.morphAttributes.normal!==void 0,morphColors:ne.morphAttributes.color!==void 0,morphTargetsCount:F,morphTextureStride:ce,numDirLights:w.directional.length,numPointLights:w.point.length,numSpotLights:w.spot.length,numSpotLightMaps:w.spotLightMap.length,numRectAreaLights:w.rectArea.length,numHemiLights:w.hemi.length,numDirLightShadows:w.directionalShadowMap.length,numPointLightShadows:w.pointShadowMap.length,numSpotLightShadows:w.spotShadowMap.length,numSpotLightShadowsWithMaps:w.numSpotLightShadowsWithMaps,numLightProbes:w.numLightProbes,numClippingPlanes:c.numPlanes,numClipIntersection:c.numIntersection,dithering:b.dithering,shadowMapEnabled:s.shadowMap.enabled&&I.length>0,shadowMapType:s.shadowMap.type,toneMapping:Ke,decodeVideoTexture:Ye&&b.map.isVideoTexture===!0&&At.getTransfer(b.map.colorSpace)===Ot,premultipliedAlpha:b.premultipliedAlpha,doubleSided:b.side===ki,flipSided:b.side===Ln,useDepthPacking:b.depthPacking>=0,depthPacking:b.depthPacking||0,index0AttributeName:b.index0AttributeName,extensionClipCullDistance:pe&&b.extensions.clipCullDistance===!0&&r.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(pe&&b.extensions.multiDraw===!0||ke)&&r.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:r.has("KHR_parallel_shader_compile"),customProgramCacheKey:b.customProgramCacheKey()};return mt.vertexUv1s=m.has(1),mt.vertexUv2s=m.has(2),mt.vertexUv3s=m.has(3),m.clear(),mt}function _(b){const w=[];if(b.shaderID?w.push(b.shaderID):(w.push(b.customVertexShaderID),w.push(b.customFragmentShaderID)),b.defines!==void 0)for(const I in b.defines)w.push(I),w.push(b.defines[I]);return b.isRawShaderMaterial===!1&&(P(w,b),R(w,b),w.push(s.outputColorSpace)),w.push(b.customProgramCacheKey),w.join()}function P(b,w){b.push(w.precision),b.push(w.outputColorSpace),b.push(w.envMapMode),b.push(w.envMapCubeUVHeight),b.push(w.mapUv),b.push(w.alphaMapUv),b.push(w.lightMapUv),b.push(w.aoMapUv),b.push(w.bumpMapUv),b.push(w.normalMapUv),b.push(w.displacementMapUv),b.push(w.emissiveMapUv),b.push(w.metalnessMapUv),b.push(w.roughnessMapUv),b.push(w.anisotropyMapUv),b.push(w.clearcoatMapUv),b.push(w.clearcoatNormalMapUv),b.push(w.clearcoatRoughnessMapUv),b.push(w.iridescenceMapUv),b.push(w.iridescenceThicknessMapUv),b.push(w.sheenColorMapUv),b.push(w.sheenRoughnessMapUv),b.push(w.specularMapUv),b.push(w.specularColorMapUv),b.push(w.specularIntensityMapUv),b.push(w.transmissionMapUv),b.push(w.thicknessMapUv),b.push(w.combine),b.push(w.fogExp2),b.push(w.sizeAttenuation),b.push(w.morphTargetsCount),b.push(w.morphAttributeCount),b.push(w.numDirLights),b.push(w.numPointLights),b.push(w.numSpotLights),b.push(w.numSpotLightMaps),b.push(w.numHemiLights),b.push(w.numRectAreaLights),b.push(w.numDirLightShadows),b.push(w.numPointLightShadows),b.push(w.numSpotLightShadows),b.push(w.numSpotLightShadowsWithMaps),b.push(w.numLightProbes),b.push(w.shadowMapType),b.push(w.toneMapping),b.push(w.numClippingPlanes),b.push(w.numClipIntersection),b.push(w.depthPacking)}function R(b,w){d.disableAll(),w.supportsVertexTextures&&d.enable(0),w.instancing&&d.enable(1),w.instancingColor&&d.enable(2),w.instancingMorph&&d.enable(3),w.matcap&&d.enable(4),w.envMap&&d.enable(5),w.normalMapObjectSpace&&d.enable(6),w.normalMapTangentSpace&&d.enable(7),w.clearcoat&&d.enable(8),w.iridescence&&d.enable(9),w.alphaTest&&d.enable(10),w.vertexColors&&d.enable(11),w.vertexAlphas&&d.enable(12),w.vertexUv1s&&d.enable(13),w.vertexUv2s&&d.enable(14),w.vertexUv3s&&d.enable(15),w.vertexTangents&&d.enable(16),w.anisotropy&&d.enable(17),w.alphaHash&&d.enable(18),w.batching&&d.enable(19),w.dispersion&&d.enable(20),w.batchingColor&&d.enable(21),b.push(d.mask),d.disableAll(),w.fog&&d.enable(0),w.useFog&&d.enable(1),w.flatShading&&d.enable(2),w.logarithmicDepthBuffer&&d.enable(3),w.skinning&&d.enable(4),w.morphTargets&&d.enable(5),w.morphNormals&&d.enable(6),w.morphColors&&d.enable(7),w.premultipliedAlpha&&d.enable(8),w.shadowMapEnabled&&d.enable(9),w.doubleSided&&d.enable(10),w.flipSided&&d.enable(11),w.useDepthPacking&&d.enable(12),w.dithering&&d.enable(13),w.transmission&&d.enable(14),w.sheen&&d.enable(15),w.opaque&&d.enable(16),w.pointsUvs&&d.enable(17),w.decodeVideoTexture&&d.enable(18),w.alphaToCoverage&&d.enable(19),b.push(d.mask)}function L(b){const w=T[b.type];let I;if(w){const Y=vi[w];I=mx.clone(Y.uniforms)}else I=b.uniforms;return I}function $(b,w){let I;for(let Y=0,K=g.length;Y<K;Y++){const oe=g[Y];if(oe.cacheKey===w){I=oe,++I.usedTimes;break}}return I===void 0&&(I=new RE(s,w,b,u),g.push(I)),I}function O(b){if(--b.usedTimes===0){const w=g.indexOf(b);g[w]=g[g.length-1],g.pop(),b.destroy()}}function D(b){h.remove(b)}function j(){h.dispose()}return{getParameters:x,getProgramCacheKey:_,getUniforms:L,acquireProgram:$,releaseProgram:O,releaseShaderCache:D,programs:g,dispose:j}}function DE(){let s=new WeakMap;function e(u){let c=s.get(u);return c===void 0&&(c={},s.set(u,c)),c}function t(u){s.delete(u)}function r(u,c,d){s.get(u)[c]=d}function o(){s=new WeakMap}return{get:e,remove:t,update:r,dispose:o}}function IE(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.material.id!==e.material.id?s.material.id-e.material.id:s.z!==e.z?s.z-e.z:s.id-e.id}function ng(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.z!==e.z?e.z-s.z:s.id-e.id}function ig(){const s=[];let e=0;const t=[],r=[],o=[];function u(){e=0,t.length=0,r.length=0,o.length=0}function c(y,v,M,T,S,x){let _=s[e];return _===void 0?(_={id:y.id,object:y,geometry:v,material:M,groupOrder:T,renderOrder:y.renderOrder,z:S,group:x},s[e]=_):(_.id=y.id,_.object=y,_.geometry=v,_.material=M,_.groupOrder=T,_.renderOrder=y.renderOrder,_.z=S,_.group=x),e++,_}function d(y,v,M,T,S,x){const _=c(y,v,M,T,S,x);M.transmission>0?r.push(_):M.transparent===!0?o.push(_):t.push(_)}function h(y,v,M,T,S,x){const _=c(y,v,M,T,S,x);M.transmission>0?r.unshift(_):M.transparent===!0?o.unshift(_):t.unshift(_)}function m(y,v){t.length>1&&t.sort(y||IE),r.length>1&&r.sort(v||ng),o.length>1&&o.sort(v||ng)}function g(){for(let y=e,v=s.length;y<v;y++){const M=s[y];if(M.id===null)break;M.id=null,M.object=null,M.geometry=null,M.material=null,M.group=null}}return{opaque:t,transmissive:r,transparent:o,init:u,push:d,unshift:h,finish:g,sort:m}}function UE(){let s=new WeakMap;function e(r,o){const u=s.get(r);let c;return u===void 0?(c=new ig,s.set(r,[c])):o>=u.length?(c=new ig,u.push(c)):c=u[o],c}function t(){s=new WeakMap}return{get:e,dispose:t}}function FE(){const s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new J,color:new _t};break;case"SpotLight":t={position:new J,direction:new J,color:new _t,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new J,color:new _t,distance:0,decay:0};break;case"HemisphereLight":t={direction:new J,skyColor:new _t,groundColor:new _t};break;case"RectAreaLight":t={color:new _t,position:new J,halfWidth:new J,halfHeight:new J};break}return s[e.id]=t,t}}}function OE(){const s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ft};break;case"SpotLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ft};break;case"PointLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ft,shadowCameraNear:1,shadowCameraFar:1e3};break}return s[e.id]=t,t}}}let kE=0;function BE(s,e){return(e.castShadow?2:0)-(s.castShadow?2:0)+(e.map?1:0)-(s.map?1:0)}function zE(s){const e=new FE,t=OE(),r={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let m=0;m<9;m++)r.probe.push(new J);const o=new J,u=new Vt,c=new Vt;function d(m){let g=0,y=0,v=0;for(let b=0;b<9;b++)r.probe[b].set(0,0,0);let M=0,T=0,S=0,x=0,_=0,P=0,R=0,L=0,$=0,O=0,D=0;m.sort(BE);for(let b=0,w=m.length;b<w;b++){const I=m[b],Y=I.color,K=I.intensity,oe=I.distance,ne=I.shadow&&I.shadow.map?I.shadow.map.texture:null;if(I.isAmbientLight)g+=Y.r*K,y+=Y.g*K,v+=Y.b*K;else if(I.isLightProbe){for(let B=0;B<9;B++)r.probe[B].addScaledVector(I.sh.coefficients[B],K);D++}else if(I.isDirectionalLight){const B=e.get(I);if(B.color.copy(I.color).multiplyScalar(I.intensity),I.castShadow){const G=I.shadow,k=t.get(I);k.shadowIntensity=G.intensity,k.shadowBias=G.bias,k.shadowNormalBias=G.normalBias,k.shadowRadius=G.radius,k.shadowMapSize=G.mapSize,r.directionalShadow[M]=k,r.directionalShadowMap[M]=ne,r.directionalShadowMatrix[M]=I.shadow.matrix,P++}r.directional[M]=B,M++}else if(I.isSpotLight){const B=e.get(I);B.position.setFromMatrixPosition(I.matrixWorld),B.color.copy(Y).multiplyScalar(K),B.distance=oe,B.coneCos=Math.cos(I.angle),B.penumbraCos=Math.cos(I.angle*(1-I.penumbra)),B.decay=I.decay,r.spot[S]=B;const G=I.shadow;if(I.map&&(r.spotLightMap[$]=I.map,$++,G.updateMatrices(I),I.castShadow&&O++),r.spotLightMatrix[S]=G.matrix,I.castShadow){const k=t.get(I);k.shadowIntensity=G.intensity,k.shadowBias=G.bias,k.shadowNormalBias=G.normalBias,k.shadowRadius=G.radius,k.shadowMapSize=G.mapSize,r.spotShadow[S]=k,r.spotShadowMap[S]=ne,L++}S++}else if(I.isRectAreaLight){const B=e.get(I);B.color.copy(Y).multiplyScalar(K),B.halfWidth.set(I.width*.5,0,0),B.halfHeight.set(0,I.height*.5,0),r.rectArea[x]=B,x++}else if(I.isPointLight){const B=e.get(I);if(B.color.copy(I.color).multiplyScalar(I.intensity),B.distance=I.distance,B.decay=I.decay,I.castShadow){const G=I.shadow,k=t.get(I);k.shadowIntensity=G.intensity,k.shadowBias=G.bias,k.shadowNormalBias=G.normalBias,k.shadowRadius=G.radius,k.shadowMapSize=G.mapSize,k.shadowCameraNear=G.camera.near,k.shadowCameraFar=G.camera.far,r.pointShadow[T]=k,r.pointShadowMap[T]=ne,r.pointShadowMatrix[T]=I.shadow.matrix,R++}r.point[T]=B,T++}else if(I.isHemisphereLight){const B=e.get(I);B.skyColor.copy(I.color).multiplyScalar(K),B.groundColor.copy(I.groundColor).multiplyScalar(K),r.hemi[_]=B,_++}}x>0&&(s.has("OES_texture_float_linear")===!0?(r.rectAreaLTC1=Pe.LTC_FLOAT_1,r.rectAreaLTC2=Pe.LTC_FLOAT_2):(r.rectAreaLTC1=Pe.LTC_HALF_1,r.rectAreaLTC2=Pe.LTC_HALF_2)),r.ambient[0]=g,r.ambient[1]=y,r.ambient[2]=v;const j=r.hash;(j.directionalLength!==M||j.pointLength!==T||j.spotLength!==S||j.rectAreaLength!==x||j.hemiLength!==_||j.numDirectionalShadows!==P||j.numPointShadows!==R||j.numSpotShadows!==L||j.numSpotMaps!==$||j.numLightProbes!==D)&&(r.directional.length=M,r.spot.length=S,r.rectArea.length=x,r.point.length=T,r.hemi.length=_,r.directionalShadow.length=P,r.directionalShadowMap.length=P,r.pointShadow.length=R,r.pointShadowMap.length=R,r.spotShadow.length=L,r.spotShadowMap.length=L,r.directionalShadowMatrix.length=P,r.pointShadowMatrix.length=R,r.spotLightMatrix.length=L+$-O,r.spotLightMap.length=$,r.numSpotLightShadowsWithMaps=O,r.numLightProbes=D,j.directionalLength=M,j.pointLength=T,j.spotLength=S,j.rectAreaLength=x,j.hemiLength=_,j.numDirectionalShadows=P,j.numPointShadows=R,j.numSpotShadows=L,j.numSpotMaps=$,j.numLightProbes=D,r.version=kE++)}function h(m,g){let y=0,v=0,M=0,T=0,S=0;const x=g.matrixWorldInverse;for(let _=0,P=m.length;_<P;_++){const R=m[_];if(R.isDirectionalLight){const L=r.directional[y];L.direction.setFromMatrixPosition(R.matrixWorld),o.setFromMatrixPosition(R.target.matrixWorld),L.direction.sub(o),L.direction.transformDirection(x),y++}else if(R.isSpotLight){const L=r.spot[M];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),L.direction.setFromMatrixPosition(R.matrixWorld),o.setFromMatrixPosition(R.target.matrixWorld),L.direction.sub(o),L.direction.transformDirection(x),M++}else if(R.isRectAreaLight){const L=r.rectArea[T];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),c.identity(),u.copy(R.matrixWorld),u.premultiply(x),c.extractRotation(u),L.halfWidth.set(R.width*.5,0,0),L.halfHeight.set(0,R.height*.5,0),L.halfWidth.applyMatrix4(c),L.halfHeight.applyMatrix4(c),T++}else if(R.isPointLight){const L=r.point[v];L.position.setFromMatrixPosition(R.matrixWorld),L.position.applyMatrix4(x),v++}else if(R.isHemisphereLight){const L=r.hemi[S];L.direction.setFromMatrixPosition(R.matrixWorld),L.direction.transformDirection(x),S++}}}return{setup:d,setupView:h,state:r}}function rg(s){const e=new zE(s),t=[],r=[];function o(g){m.camera=g,t.length=0,r.length=0}function u(g){t.push(g)}function c(g){r.push(g)}function d(){e.setup(t)}function h(g){e.setupView(t,g)}const m={lightsArray:t,shadowsArray:r,camera:null,lights:e,transmissionRenderTarget:{}};return{init:o,state:m,setupLights:d,setupLightsView:h,pushLight:u,pushShadow:c}}function HE(s){let e=new WeakMap;function t(o,u=0){const c=e.get(o);let d;return c===void 0?(d=new rg(s),e.set(o,[d])):u>=c.length?(d=new rg(s),c.push(d)):d=c[u],d}function r(){e=new WeakMap}return{get:t,dispose:r}}class VE extends ns{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=B0,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}}class GE extends ns{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}}const WE=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,XE=`uniform sampler2D shadow_pass;
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
}`;function jE(s,e,t){let r=new wd;const o=new ft,u=new ft,c=new Zt,d=new VE({depthPacking:z0}),h=new GE,m={},g=t.maxTextureSize,y={[Mr]:Ln,[Ln]:Mr,[ki]:ki},v=new Er({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new ft},radius:{value:4}},vertexShader:WE,fragmentShader:XE}),M=v.clone();M.defines.HORIZONTAL_PASS=1;const T=new Hn;T.setAttribute("position",new di(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const S=new zn(T,v),x=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=wg;let _=this.type;this.render=function(O,D,j){if(x.enabled===!1||x.autoUpdate===!1&&x.needsUpdate===!1||O.length===0)return;const b=s.getRenderTarget(),w=s.getActiveCubeFace(),I=s.getActiveMipmapLevel(),Y=s.state;Y.setBlending(xr),Y.buffers.color.setClear(1,1,1,1),Y.buffers.depth.setTest(!0),Y.setScissorTest(!1);const K=_!==Oi&&this.type===Oi,oe=_===Oi&&this.type!==Oi;for(let ne=0,B=O.length;ne<B;ne++){const G=O[ne],k=G.shadow;if(k===void 0){console.warn("THREE.WebGLShadowMap:",G,"has no shadow.");continue}if(k.autoUpdate===!1&&k.needsUpdate===!1)continue;o.copy(k.mapSize);const ue=k.getFrameExtents();if(o.multiply(ue),u.copy(k.mapSize),(o.x>g||o.y>g)&&(o.x>g&&(u.x=Math.floor(g/ue.x),o.x=u.x*ue.x,k.mapSize.x=u.x),o.y>g&&(u.y=Math.floor(g/ue.y),o.y=u.y*ue.y,k.mapSize.y=u.y)),k.map===null||K===!0||oe===!0){const F=this.type!==Oi?{minFilter:Kn,magFilter:Kn}:{};k.map!==null&&k.map.dispose(),k.map=new es(o.x,o.y,F),k.map.texture.name=G.name+".shadowMap",k.camera.updateProjectionMatrix()}s.setRenderTarget(k.map),s.clear();const le=k.getViewportCount();for(let F=0;F<le;F++){const ce=k.getViewport(F);c.set(u.x*ce.x,u.y*ce.y,u.x*ce.z,u.y*ce.w),Y.viewport(c),k.updateMatrices(G,F),r=k.getFrustum(),L(D,j,k.camera,G,this.type)}k.isPointLightShadow!==!0&&this.type===Oi&&P(k,j),k.needsUpdate=!1}_=this.type,x.needsUpdate=!1,s.setRenderTarget(b,w,I)};function P(O,D){const j=e.update(S);v.defines.VSM_SAMPLES!==O.blurSamples&&(v.defines.VSM_SAMPLES=O.blurSamples,M.defines.VSM_SAMPLES=O.blurSamples,v.needsUpdate=!0,M.needsUpdate=!0),O.mapPass===null&&(O.mapPass=new es(o.x,o.y)),v.uniforms.shadow_pass.value=O.map.texture,v.uniforms.resolution.value=O.mapSize,v.uniforms.radius.value=O.radius,s.setRenderTarget(O.mapPass),s.clear(),s.renderBufferDirect(D,null,j,v,S,null),M.uniforms.shadow_pass.value=O.mapPass.texture,M.uniforms.resolution.value=O.mapSize,M.uniforms.radius.value=O.radius,s.setRenderTarget(O.map),s.clear(),s.renderBufferDirect(D,null,j,M,S,null)}function R(O,D,j,b){let w=null;const I=j.isPointLight===!0?O.customDistanceMaterial:O.customDepthMaterial;if(I!==void 0)w=I;else if(w=j.isPointLight===!0?h:d,s.localClippingEnabled&&D.clipShadows===!0&&Array.isArray(D.clippingPlanes)&&D.clippingPlanes.length!==0||D.displacementMap&&D.displacementScale!==0||D.alphaMap&&D.alphaTest>0||D.map&&D.alphaTest>0){const Y=w.uuid,K=D.uuid;let oe=m[Y];oe===void 0&&(oe={},m[Y]=oe);let ne=oe[K];ne===void 0&&(ne=w.clone(),oe[K]=ne,D.addEventListener("dispose",$)),w=ne}if(w.visible=D.visible,w.wireframe=D.wireframe,b===Oi?w.side=D.shadowSide!==null?D.shadowSide:D.side:w.side=D.shadowSide!==null?D.shadowSide:y[D.side],w.alphaMap=D.alphaMap,w.alphaTest=D.alphaTest,w.map=D.map,w.clipShadows=D.clipShadows,w.clippingPlanes=D.clippingPlanes,w.clipIntersection=D.clipIntersection,w.displacementMap=D.displacementMap,w.displacementScale=D.displacementScale,w.displacementBias=D.displacementBias,w.wireframeLinewidth=D.wireframeLinewidth,w.linewidth=D.linewidth,j.isPointLight===!0&&w.isMeshDistanceMaterial===!0){const Y=s.properties.get(w);Y.light=j}return w}function L(O,D,j,b,w){if(O.visible===!1)return;if(O.layers.test(D.layers)&&(O.isMesh||O.isLine||O.isPoints)&&(O.castShadow||O.receiveShadow&&w===Oi)&&(!O.frustumCulled||r.intersectsObject(O))){O.modelViewMatrix.multiplyMatrices(j.matrixWorldInverse,O.matrixWorld);const K=e.update(O),oe=O.material;if(Array.isArray(oe)){const ne=K.groups;for(let B=0,G=ne.length;B<G;B++){const k=ne[B],ue=oe[k.materialIndex];if(ue&&ue.visible){const le=R(O,ue,b,w);O.onBeforeShadow(s,O,D,j,K,le,k),s.renderBufferDirect(j,null,K,le,O,k),O.onAfterShadow(s,O,D,j,K,le,k)}}}else if(oe.visible){const ne=R(O,oe,b,w);O.onBeforeShadow(s,O,D,j,K,ne,null),s.renderBufferDirect(j,null,K,ne,O,null),O.onAfterShadow(s,O,D,j,K,ne,null)}}const Y=O.children;for(let K=0,oe=Y.length;K<oe;K++)L(Y[K],D,j,b,w)}function $(O){O.target.removeEventListener("dispose",$);for(const j in m){const b=m[j],w=O.target.uuid;w in b&&(b[w].dispose(),delete b[w])}}}function YE(s){function e(){let X=!1;const we=new Zt;let de=null;const he=new Zt(0,0,0,0);return{setMask:function(Te){de!==Te&&!X&&(s.colorMask(Te,Te,Te,Te),de=Te)},setLocked:function(Te){X=Te},setClear:function(Te,qe,pe,Ke,mt){mt===!0&&(Te*=Ke,qe*=Ke,pe*=Ke),we.set(Te,qe,pe,Ke),he.equals(we)===!1&&(s.clearColor(Te,qe,pe,Ke),he.copy(we))},reset:function(){X=!1,de=null,he.set(-1,0,0,0)}}}function t(){let X=!1,we=null,de=null,he=null;return{setTest:function(Te){Te?xe(s.DEPTH_TEST):Me(s.DEPTH_TEST)},setMask:function(Te){we!==Te&&!X&&(s.depthMask(Te),we=Te)},setFunc:function(Te){if(de!==Te){switch(Te){case M0:s.depthFunc(s.NEVER);break;case E0:s.depthFunc(s.ALWAYS);break;case w0:s.depthFunc(s.LESS);break;case jl:s.depthFunc(s.LEQUAL);break;case T0:s.depthFunc(s.EQUAL);break;case A0:s.depthFunc(s.GEQUAL);break;case C0:s.depthFunc(s.GREATER);break;case R0:s.depthFunc(s.NOTEQUAL);break;default:s.depthFunc(s.LEQUAL)}de=Te}},setLocked:function(Te){X=Te},setClear:function(Te){he!==Te&&(s.clearDepth(Te),he=Te)},reset:function(){X=!1,we=null,de=null,he=null}}}function r(){let X=!1,we=null,de=null,he=null,Te=null,qe=null,pe=null,Ke=null,mt=null;return{setTest:function(ut){X||(ut?xe(s.STENCIL_TEST):Me(s.STENCIL_TEST))},setMask:function(ut){we!==ut&&!X&&(s.stencilMask(ut),we=ut)},setFunc:function(ut,Yt,qt){(de!==ut||he!==Yt||Te!==qt)&&(s.stencilFunc(ut,Yt,qt),de=ut,he=Yt,Te=qt)},setOp:function(ut,Yt,qt){(qe!==ut||pe!==Yt||Ke!==qt)&&(s.stencilOp(ut,Yt,qt),qe=ut,pe=Yt,Ke=qt)},setLocked:function(ut){X=ut},setClear:function(ut){mt!==ut&&(s.clearStencil(ut),mt=ut)},reset:function(){X=!1,we=null,de=null,he=null,Te=null,qe=null,pe=null,Ke=null,mt=null}}}const o=new e,u=new t,c=new r,d=new WeakMap,h=new WeakMap;let m={},g={},y=new WeakMap,v=[],M=null,T=!1,S=null,x=null,_=null,P=null,R=null,L=null,$=null,O=new _t(0,0,0),D=0,j=!1,b=null,w=null,I=null,Y=null,K=null;const oe=s.getParameter(s.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let ne=!1,B=0;const G=s.getParameter(s.VERSION);G.indexOf("WebGL")!==-1?(B=parseFloat(/^WebGL (\d)/.exec(G)[1]),ne=B>=1):G.indexOf("OpenGL ES")!==-1&&(B=parseFloat(/^OpenGL ES (\d)/.exec(G)[1]),ne=B>=2);let k=null,ue={};const le=s.getParameter(s.SCISSOR_BOX),F=s.getParameter(s.VIEWPORT),ce=new Zt().fromArray(le),Ie=new Zt().fromArray(F);function te(X,we,de,he){const Te=new Uint8Array(4),qe=s.createTexture();s.bindTexture(X,qe),s.texParameteri(X,s.TEXTURE_MIN_FILTER,s.NEAREST),s.texParameteri(X,s.TEXTURE_MAG_FILTER,s.NEAREST);for(let pe=0;pe<de;pe++)X===s.TEXTURE_3D||X===s.TEXTURE_2D_ARRAY?s.texImage3D(we,0,s.RGBA,1,1,he,0,s.RGBA,s.UNSIGNED_BYTE,Te):s.texImage2D(we+pe,0,s.RGBA,1,1,0,s.RGBA,s.UNSIGNED_BYTE,Te);return qe}const fe={};fe[s.TEXTURE_2D]=te(s.TEXTURE_2D,s.TEXTURE_2D,1),fe[s.TEXTURE_CUBE_MAP]=te(s.TEXTURE_CUBE_MAP,s.TEXTURE_CUBE_MAP_POSITIVE_X,6),fe[s.TEXTURE_2D_ARRAY]=te(s.TEXTURE_2D_ARRAY,s.TEXTURE_2D_ARRAY,1,1),fe[s.TEXTURE_3D]=te(s.TEXTURE_3D,s.TEXTURE_3D,1,1),o.setClear(0,0,0,1),u.setClear(1),c.setClear(0),xe(s.DEPTH_TEST),u.setFunc(jl),yt(!1),We(fm),xe(s.CULL_FACE),bt(xr);function xe(X){m[X]!==!0&&(s.enable(X),m[X]=!0)}function Me(X){m[X]!==!1&&(s.disable(X),m[X]=!1)}function Le(X,we){return g[X]!==we?(s.bindFramebuffer(X,we),g[X]=we,X===s.DRAW_FRAMEBUFFER&&(g[s.FRAMEBUFFER]=we),X===s.FRAMEBUFFER&&(g[s.DRAW_FRAMEBUFFER]=we),!0):!1}function ke(X,we){let de=v,he=!1;if(X){de=y.get(we),de===void 0&&(de=[],y.set(we,de));const Te=X.textures;if(de.length!==Te.length||de[0]!==s.COLOR_ATTACHMENT0){for(let qe=0,pe=Te.length;qe<pe;qe++)de[qe]=s.COLOR_ATTACHMENT0+qe;de.length=Te.length,he=!0}}else de[0]!==s.BACK&&(de[0]=s.BACK,he=!0);he&&s.drawBuffers(de)}function Ye(X){return M!==X?(s.useProgram(X),M=X,!0):!1}const wt={[Yr]:s.FUNC_ADD,[s0]:s.FUNC_SUBTRACT,[a0]:s.FUNC_REVERSE_SUBTRACT};wt[o0]=s.MIN,wt[l0]=s.MAX;const z={[u0]:s.ZERO,[c0]:s.ONE,[f0]:s.SRC_COLOR,[Lf]:s.SRC_ALPHA,[_0]:s.SRC_ALPHA_SATURATE,[m0]:s.DST_COLOR,[h0]:s.DST_ALPHA,[d0]:s.ONE_MINUS_SRC_COLOR,[Nf]:s.ONE_MINUS_SRC_ALPHA,[g0]:s.ONE_MINUS_DST_COLOR,[p0]:s.ONE_MINUS_DST_ALPHA,[v0]:s.CONSTANT_COLOR,[x0]:s.ONE_MINUS_CONSTANT_COLOR,[y0]:s.CONSTANT_ALPHA,[S0]:s.ONE_MINUS_CONSTANT_ALPHA};function bt(X,we,de,he,Te,qe,pe,Ke,mt,ut){if(X===xr){T===!0&&(Me(s.BLEND),T=!1);return}if(T===!1&&(xe(s.BLEND),T=!0),X!==r0){if(X!==S||ut!==j){if((x!==Yr||R!==Yr)&&(s.blendEquation(s.FUNC_ADD),x=Yr,R=Yr),ut)switch(X){case Zs:s.blendFuncSeparate(s.ONE,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case dm:s.blendFunc(s.ONE,s.ONE);break;case hm:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case pm:s.blendFuncSeparate(s.ZERO,s.SRC_COLOR,s.ZERO,s.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",X);break}else switch(X){case Zs:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case dm:s.blendFunc(s.SRC_ALPHA,s.ONE);break;case hm:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case pm:s.blendFunc(s.ZERO,s.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",X);break}_=null,P=null,L=null,$=null,O.set(0,0,0),D=0,S=X,j=ut}return}Te=Te||we,qe=qe||de,pe=pe||he,(we!==x||Te!==R)&&(s.blendEquationSeparate(wt[we],wt[Te]),x=we,R=Te),(de!==_||he!==P||qe!==L||pe!==$)&&(s.blendFuncSeparate(z[de],z[he],z[qe],z[pe]),_=de,P=he,L=qe,$=pe),(Ke.equals(O)===!1||mt!==D)&&(s.blendColor(Ke.r,Ke.g,Ke.b,mt),O.copy(Ke),D=mt),S=X,j=!1}function vt(X,we){X.side===ki?Me(s.CULL_FACE):xe(s.CULL_FACE);let de=X.side===Ln;we&&(de=!de),yt(de),X.blending===Zs&&X.transparent===!1?bt(xr):bt(X.blending,X.blendEquation,X.blendSrc,X.blendDst,X.blendEquationAlpha,X.blendSrcAlpha,X.blendDstAlpha,X.blendColor,X.blendAlpha,X.premultipliedAlpha),u.setFunc(X.depthFunc),u.setTest(X.depthTest),u.setMask(X.depthWrite),o.setMask(X.colorWrite);const he=X.stencilWrite;c.setTest(he),he&&(c.setMask(X.stencilWriteMask),c.setFunc(X.stencilFunc,X.stencilRef,X.stencilFuncMask),c.setOp(X.stencilFail,X.stencilZFail,X.stencilZPass)),tt(X.polygonOffset,X.polygonOffsetFactor,X.polygonOffsetUnits),X.alphaToCoverage===!0?xe(s.SAMPLE_ALPHA_TO_COVERAGE):Me(s.SAMPLE_ALPHA_TO_COVERAGE)}function yt(X){b!==X&&(X?s.frontFace(s.CW):s.frontFace(s.CCW),b=X)}function We(X){X!==t0?(xe(s.CULL_FACE),X!==w&&(X===fm?s.cullFace(s.BACK):X===n0?s.cullFace(s.FRONT):s.cullFace(s.FRONT_AND_BACK))):Me(s.CULL_FACE),w=X}function Lt(X){X!==I&&(ne&&s.lineWidth(X),I=X)}function tt(X,we,de){X?(xe(s.POLYGON_OFFSET_FILL),(Y!==we||K!==de)&&(s.polygonOffset(we,de),Y=we,K=de)):Me(s.POLYGON_OFFSET_FILL)}function rt(X){X?xe(s.SCISSOR_TEST):Me(s.SCISSOR_TEST)}function U(X){X===void 0&&(X=s.TEXTURE0+oe-1),k!==X&&(s.activeTexture(X),k=X)}function A(X,we,de){de===void 0&&(k===null?de=s.TEXTURE0+oe-1:de=k);let he=ue[de];he===void 0&&(he={type:void 0,texture:void 0},ue[de]=he),(he.type!==X||he.texture!==we)&&(k!==de&&(s.activeTexture(de),k=de),s.bindTexture(X,we||fe[X]),he.type=X,he.texture=we)}function se(){const X=ue[k];X!==void 0&&X.type!==void 0&&(s.bindTexture(X.type,null),X.type=void 0,X.texture=void 0)}function _e(){try{s.compressedTexImage2D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function ye(){try{s.compressedTexImage3D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function me(){try{s.texSubImage2D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function je(){try{s.texSubImage3D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function be(){try{s.compressedTexSubImage2D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function De(){try{s.compressedTexSubImage3D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function ot(){try{s.texStorage2D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function Ee(){try{s.texStorage3D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function Ne(){try{s.texImage2D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function pt(){try{s.texImage3D.apply(s,arguments)}catch(X){console.error("THREE.WebGLState:",X)}}function Je(X){ce.equals(X)===!1&&(s.scissor(X.x,X.y,X.z,X.w),ce.copy(X))}function Oe(X){Ie.equals(X)===!1&&(s.viewport(X.x,X.y,X.z,X.w),Ie.copy(X))}function st(X,we){let de=h.get(we);de===void 0&&(de=new WeakMap,h.set(we,de));let he=de.get(X);he===void 0&&(he=s.getUniformBlockIndex(we,X.name),de.set(X,he))}function lt(X,we){const he=h.get(we).get(X);d.get(we)!==he&&(s.uniformBlockBinding(we,he,X.__bindingPointIndex),d.set(we,he))}function Tt(){s.disable(s.BLEND),s.disable(s.CULL_FACE),s.disable(s.DEPTH_TEST),s.disable(s.POLYGON_OFFSET_FILL),s.disable(s.SCISSOR_TEST),s.disable(s.STENCIL_TEST),s.disable(s.SAMPLE_ALPHA_TO_COVERAGE),s.blendEquation(s.FUNC_ADD),s.blendFunc(s.ONE,s.ZERO),s.blendFuncSeparate(s.ONE,s.ZERO,s.ONE,s.ZERO),s.blendColor(0,0,0,0),s.colorMask(!0,!0,!0,!0),s.clearColor(0,0,0,0),s.depthMask(!0),s.depthFunc(s.LESS),s.clearDepth(1),s.stencilMask(4294967295),s.stencilFunc(s.ALWAYS,0,4294967295),s.stencilOp(s.KEEP,s.KEEP,s.KEEP),s.clearStencil(0),s.cullFace(s.BACK),s.frontFace(s.CCW),s.polygonOffset(0,0),s.activeTexture(s.TEXTURE0),s.bindFramebuffer(s.FRAMEBUFFER,null),s.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),s.bindFramebuffer(s.READ_FRAMEBUFFER,null),s.useProgram(null),s.lineWidth(1),s.scissor(0,0,s.canvas.width,s.canvas.height),s.viewport(0,0,s.canvas.width,s.canvas.height),m={},k=null,ue={},g={},y=new WeakMap,v=[],M=null,T=!1,S=null,x=null,_=null,P=null,R=null,L=null,$=null,O=new _t(0,0,0),D=0,j=!1,b=null,w=null,I=null,Y=null,K=null,ce.set(0,0,s.canvas.width,s.canvas.height),Ie.set(0,0,s.canvas.width,s.canvas.height),o.reset(),u.reset(),c.reset()}return{buffers:{color:o,depth:u,stencil:c},enable:xe,disable:Me,bindFramebuffer:Le,drawBuffers:ke,useProgram:Ye,setBlending:bt,setMaterial:vt,setFlipSided:yt,setCullFace:We,setLineWidth:Lt,setPolygonOffset:tt,setScissorTest:rt,activeTexture:U,bindTexture:A,unbindTexture:se,compressedTexImage2D:_e,compressedTexImage3D:ye,texImage2D:Ne,texImage3D:pt,updateUBOMapping:st,uniformBlockBinding:lt,texStorage2D:ot,texStorage3D:Ee,texSubImage2D:me,texSubImage3D:je,compressedTexSubImage2D:be,compressedTexSubImage3D:De,scissor:Je,viewport:Oe,reset:Tt}}function sg(s,e,t,r){const o=qE(r);switch(t){case Pg:return s*e;case Ng:return s*e;case Dg:return s*e*2;case Ig:return s*e/o.components*o.byteLength;case xd:return s*e/o.components*o.byteLength;case Ug:return s*e*2/o.components*o.byteLength;case yd:return s*e*2/o.components*o.byteLength;case Lg:return s*e*3/o.components*o.byteLength;case fi:return s*e*4/o.components*o.byteLength;case Sd:return s*e*4/o.components*o.byteLength;case zl:case Hl:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case Vl:case Gl:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case kf:case zf:return Math.max(s,16)*Math.max(e,8)/4;case Of:case Bf:return Math.max(s,8)*Math.max(e,8)/2;case Hf:case Vf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case Gf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case Wf:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case Xf:return Math.floor((s+4)/5)*Math.floor((e+3)/4)*16;case jf:return Math.floor((s+4)/5)*Math.floor((e+4)/5)*16;case Yf:return Math.floor((s+5)/6)*Math.floor((e+4)/5)*16;case qf:return Math.floor((s+5)/6)*Math.floor((e+5)/6)*16;case $f:return Math.floor((s+7)/8)*Math.floor((e+4)/5)*16;case Kf:return Math.floor((s+7)/8)*Math.floor((e+5)/6)*16;case Zf:return Math.floor((s+7)/8)*Math.floor((e+7)/8)*16;case Qf:return Math.floor((s+9)/10)*Math.floor((e+4)/5)*16;case Jf:return Math.floor((s+9)/10)*Math.floor((e+5)/6)*16;case ed:return Math.floor((s+9)/10)*Math.floor((e+7)/8)*16;case td:return Math.floor((s+9)/10)*Math.floor((e+9)/10)*16;case nd:return Math.floor((s+11)/12)*Math.floor((e+9)/10)*16;case id:return Math.floor((s+11)/12)*Math.floor((e+11)/12)*16;case Wl:case rd:case sd:return Math.ceil(s/4)*Math.ceil(e/4)*16;case Fg:case ad:return Math.ceil(s/4)*Math.ceil(e/4)*8;case od:case ld:return Math.ceil(s/4)*Math.ceil(e/4)*16}throw new Error(`Unable to determine texture byte length for ${t} format.`)}function qE(s){switch(s){case Hi:case Cg:return{byteLength:1,components:1};case Qa:case Rg:case eo:return{byteLength:2,components:1};case _d:case vd:return{byteLength:2,components:4};case Jr:case gd:case Bi:return{byteLength:4,components:1};case bg:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${s}.`)}function $E(s,e,t,r,o,u,c){const d=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,h=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),m=new ft,g=new WeakMap;let y;const v=new WeakMap;let M=!1;try{M=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function T(U,A){return M?new OffscreenCanvas(U,A):Zl("canvas")}function S(U,A,se){let _e=1;const ye=rt(U);if((ye.width>se||ye.height>se)&&(_e=se/Math.max(ye.width,ye.height)),_e<1)if(typeof HTMLImageElement<"u"&&U instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&U instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&U instanceof ImageBitmap||typeof VideoFrame<"u"&&U instanceof VideoFrame){const me=Math.floor(_e*ye.width),je=Math.floor(_e*ye.height);y===void 0&&(y=T(me,je));const be=A?T(me,je):y;return be.width=me,be.height=je,be.getContext("2d").drawImage(U,0,0,me,je),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+ye.width+"x"+ye.height+") to ("+me+"x"+je+")."),be}else return"data"in U&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+ye.width+"x"+ye.height+")."),U;return U}function x(U){return U.generateMipmaps&&U.minFilter!==Kn&&U.minFilter!==ui}function _(U){s.generateMipmap(U)}function P(U,A,se,_e,ye=!1){if(U!==null){if(s[U]!==void 0)return s[U];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+U+"'")}let me=A;if(A===s.RED&&(se===s.FLOAT&&(me=s.R32F),se===s.HALF_FLOAT&&(me=s.R16F),se===s.UNSIGNED_BYTE&&(me=s.R8)),A===s.RED_INTEGER&&(se===s.UNSIGNED_BYTE&&(me=s.R8UI),se===s.UNSIGNED_SHORT&&(me=s.R16UI),se===s.UNSIGNED_INT&&(me=s.R32UI),se===s.BYTE&&(me=s.R8I),se===s.SHORT&&(me=s.R16I),se===s.INT&&(me=s.R32I)),A===s.RG&&(se===s.FLOAT&&(me=s.RG32F),se===s.HALF_FLOAT&&(me=s.RG16F),se===s.UNSIGNED_BYTE&&(me=s.RG8)),A===s.RG_INTEGER&&(se===s.UNSIGNED_BYTE&&(me=s.RG8UI),se===s.UNSIGNED_SHORT&&(me=s.RG16UI),se===s.UNSIGNED_INT&&(me=s.RG32UI),se===s.BYTE&&(me=s.RG8I),se===s.SHORT&&(me=s.RG16I),se===s.INT&&(me=s.RG32I)),A===s.RGB&&se===s.UNSIGNED_INT_5_9_9_9_REV&&(me=s.RGB9_E5),A===s.RGBA){const je=ye?Yl:At.getTransfer(_e);se===s.FLOAT&&(me=s.RGBA32F),se===s.HALF_FLOAT&&(me=s.RGBA16F),se===s.UNSIGNED_BYTE&&(me=je===Ot?s.SRGB8_ALPHA8:s.RGBA8),se===s.UNSIGNED_SHORT_4_4_4_4&&(me=s.RGBA4),se===s.UNSIGNED_SHORT_5_5_5_1&&(me=s.RGB5_A1)}return(me===s.R16F||me===s.R32F||me===s.RG16F||me===s.RG32F||me===s.RGBA16F||me===s.RGBA32F)&&e.get("EXT_color_buffer_float"),me}function R(U,A){let se;return U?A===null||A===Jr||A===ia?se=s.DEPTH24_STENCIL8:A===Bi?se=s.DEPTH32F_STENCIL8:A===Qa&&(se=s.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):A===null||A===Jr||A===ia?se=s.DEPTH_COMPONENT24:A===Bi?se=s.DEPTH_COMPONENT32F:A===Qa&&(se=s.DEPTH_COMPONENT16),se}function L(U,A){return x(U)===!0||U.isFramebufferTexture&&U.minFilter!==Kn&&U.minFilter!==ui?Math.log2(Math.max(A.width,A.height))+1:U.mipmaps!==void 0&&U.mipmaps.length>0?U.mipmaps.length:U.isCompressedTexture&&Array.isArray(U.image)?A.mipmaps.length:1}function $(U){const A=U.target;A.removeEventListener("dispose",$),D(A),A.isVideoTexture&&g.delete(A)}function O(U){const A=U.target;A.removeEventListener("dispose",O),b(A)}function D(U){const A=r.get(U);if(A.__webglInit===void 0)return;const se=U.source,_e=v.get(se);if(_e){const ye=_e[A.__cacheKey];ye.usedTimes--,ye.usedTimes===0&&j(U),Object.keys(_e).length===0&&v.delete(se)}r.remove(U)}function j(U){const A=r.get(U);s.deleteTexture(A.__webglTexture);const se=U.source,_e=v.get(se);delete _e[A.__cacheKey],c.memory.textures--}function b(U){const A=r.get(U);if(U.depthTexture&&U.depthTexture.dispose(),U.isWebGLCubeRenderTarget)for(let _e=0;_e<6;_e++){if(Array.isArray(A.__webglFramebuffer[_e]))for(let ye=0;ye<A.__webglFramebuffer[_e].length;ye++)s.deleteFramebuffer(A.__webglFramebuffer[_e][ye]);else s.deleteFramebuffer(A.__webglFramebuffer[_e]);A.__webglDepthbuffer&&s.deleteRenderbuffer(A.__webglDepthbuffer[_e])}else{if(Array.isArray(A.__webglFramebuffer))for(let _e=0;_e<A.__webglFramebuffer.length;_e++)s.deleteFramebuffer(A.__webglFramebuffer[_e]);else s.deleteFramebuffer(A.__webglFramebuffer);if(A.__webglDepthbuffer&&s.deleteRenderbuffer(A.__webglDepthbuffer),A.__webglMultisampledFramebuffer&&s.deleteFramebuffer(A.__webglMultisampledFramebuffer),A.__webglColorRenderbuffer)for(let _e=0;_e<A.__webglColorRenderbuffer.length;_e++)A.__webglColorRenderbuffer[_e]&&s.deleteRenderbuffer(A.__webglColorRenderbuffer[_e]);A.__webglDepthRenderbuffer&&s.deleteRenderbuffer(A.__webglDepthRenderbuffer)}const se=U.textures;for(let _e=0,ye=se.length;_e<ye;_e++){const me=r.get(se[_e]);me.__webglTexture&&(s.deleteTexture(me.__webglTexture),c.memory.textures--),r.remove(se[_e])}r.remove(U)}let w=0;function I(){w=0}function Y(){const U=w;return U>=o.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+U+" texture units while this GPU supports only "+o.maxTextures),w+=1,U}function K(U){const A=[];return A.push(U.wrapS),A.push(U.wrapT),A.push(U.wrapR||0),A.push(U.magFilter),A.push(U.minFilter),A.push(U.anisotropy),A.push(U.internalFormat),A.push(U.format),A.push(U.type),A.push(U.generateMipmaps),A.push(U.premultiplyAlpha),A.push(U.flipY),A.push(U.unpackAlignment),A.push(U.colorSpace),A.join()}function oe(U,A){const se=r.get(U);if(U.isVideoTexture&&Lt(U),U.isRenderTargetTexture===!1&&U.version>0&&se.__version!==U.version){const _e=U.image;if(_e===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(_e.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{Ie(se,U,A);return}}t.bindTexture(s.TEXTURE_2D,se.__webglTexture,s.TEXTURE0+A)}function ne(U,A){const se=r.get(U);if(U.version>0&&se.__version!==U.version){Ie(se,U,A);return}t.bindTexture(s.TEXTURE_2D_ARRAY,se.__webglTexture,s.TEXTURE0+A)}function B(U,A){const se=r.get(U);if(U.version>0&&se.__version!==U.version){Ie(se,U,A);return}t.bindTexture(s.TEXTURE_3D,se.__webglTexture,s.TEXTURE0+A)}function G(U,A){const se=r.get(U);if(U.version>0&&se.__version!==U.version){te(se,U,A);return}t.bindTexture(s.TEXTURE_CUBE_MAP,se.__webglTexture,s.TEXTURE0+A)}const k={[Uf]:s.REPEAT,[$r]:s.CLAMP_TO_EDGE,[Ff]:s.MIRRORED_REPEAT},ue={[Kn]:s.NEAREST,[k0]:s.NEAREST_MIPMAP_NEAREST,[cl]:s.NEAREST_MIPMAP_LINEAR,[ui]:s.LINEAR,[Xc]:s.LINEAR_MIPMAP_NEAREST,[Kr]:s.LINEAR_MIPMAP_LINEAR},le={[V0]:s.NEVER,[q0]:s.ALWAYS,[G0]:s.LESS,[kg]:s.LEQUAL,[W0]:s.EQUAL,[Y0]:s.GEQUAL,[X0]:s.GREATER,[j0]:s.NOTEQUAL};function F(U,A){if(A.type===Bi&&e.has("OES_texture_float_linear")===!1&&(A.magFilter===ui||A.magFilter===Xc||A.magFilter===cl||A.magFilter===Kr||A.minFilter===ui||A.minFilter===Xc||A.minFilter===cl||A.minFilter===Kr)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),s.texParameteri(U,s.TEXTURE_WRAP_S,k[A.wrapS]),s.texParameteri(U,s.TEXTURE_WRAP_T,k[A.wrapT]),(U===s.TEXTURE_3D||U===s.TEXTURE_2D_ARRAY)&&s.texParameteri(U,s.TEXTURE_WRAP_R,k[A.wrapR]),s.texParameteri(U,s.TEXTURE_MAG_FILTER,ue[A.magFilter]),s.texParameteri(U,s.TEXTURE_MIN_FILTER,ue[A.minFilter]),A.compareFunction&&(s.texParameteri(U,s.TEXTURE_COMPARE_MODE,s.COMPARE_REF_TO_TEXTURE),s.texParameteri(U,s.TEXTURE_COMPARE_FUNC,le[A.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(A.magFilter===Kn||A.minFilter!==cl&&A.minFilter!==Kr||A.type===Bi&&e.has("OES_texture_float_linear")===!1)return;if(A.anisotropy>1||r.get(A).__currentAnisotropy){const se=e.get("EXT_texture_filter_anisotropic");s.texParameterf(U,se.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(A.anisotropy,o.getMaxAnisotropy())),r.get(A).__currentAnisotropy=A.anisotropy}}}function ce(U,A){let se=!1;U.__webglInit===void 0&&(U.__webglInit=!0,A.addEventListener("dispose",$));const _e=A.source;let ye=v.get(_e);ye===void 0&&(ye={},v.set(_e,ye));const me=K(A);if(me!==U.__cacheKey){ye[me]===void 0&&(ye[me]={texture:s.createTexture(),usedTimes:0},c.memory.textures++,se=!0),ye[me].usedTimes++;const je=ye[U.__cacheKey];je!==void 0&&(ye[U.__cacheKey].usedTimes--,je.usedTimes===0&&j(A)),U.__cacheKey=me,U.__webglTexture=ye[me].texture}return se}function Ie(U,A,se){let _e=s.TEXTURE_2D;(A.isDataArrayTexture||A.isCompressedArrayTexture)&&(_e=s.TEXTURE_2D_ARRAY),A.isData3DTexture&&(_e=s.TEXTURE_3D);const ye=ce(U,A),me=A.source;t.bindTexture(_e,U.__webglTexture,s.TEXTURE0+se);const je=r.get(me);if(me.version!==je.__version||ye===!0){t.activeTexture(s.TEXTURE0+se);const be=At.getPrimaries(At.workingColorSpace),De=A.colorSpace===vr?null:At.getPrimaries(A.colorSpace),ot=A.colorSpace===vr||be===De?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,A.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,A.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,A.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,ot);let Ee=S(A.image,!1,o.maxTextureSize);Ee=tt(A,Ee);const Ne=u.convert(A.format,A.colorSpace),pt=u.convert(A.type);let Je=P(A.internalFormat,Ne,pt,A.colorSpace,A.isVideoTexture);F(_e,A);let Oe;const st=A.mipmaps,lt=A.isVideoTexture!==!0,Tt=je.__version===void 0||ye===!0,X=me.dataReady,we=L(A,Ee);if(A.isDepthTexture)Je=R(A.format===ra,A.type),Tt&&(lt?t.texStorage2D(s.TEXTURE_2D,1,Je,Ee.width,Ee.height):t.texImage2D(s.TEXTURE_2D,0,Je,Ee.width,Ee.height,0,Ne,pt,null));else if(A.isDataTexture)if(st.length>0){lt&&Tt&&t.texStorage2D(s.TEXTURE_2D,we,Je,st[0].width,st[0].height);for(let de=0,he=st.length;de<he;de++)Oe=st[de],lt?X&&t.texSubImage2D(s.TEXTURE_2D,de,0,0,Oe.width,Oe.height,Ne,pt,Oe.data):t.texImage2D(s.TEXTURE_2D,de,Je,Oe.width,Oe.height,0,Ne,pt,Oe.data);A.generateMipmaps=!1}else lt?(Tt&&t.texStorage2D(s.TEXTURE_2D,we,Je,Ee.width,Ee.height),X&&t.texSubImage2D(s.TEXTURE_2D,0,0,0,Ee.width,Ee.height,Ne,pt,Ee.data)):t.texImage2D(s.TEXTURE_2D,0,Je,Ee.width,Ee.height,0,Ne,pt,Ee.data);else if(A.isCompressedTexture)if(A.isCompressedArrayTexture){lt&&Tt&&t.texStorage3D(s.TEXTURE_2D_ARRAY,we,Je,st[0].width,st[0].height,Ee.depth);for(let de=0,he=st.length;de<he;de++)if(Oe=st[de],A.format!==fi)if(Ne!==null)if(lt){if(X)if(A.layerUpdates.size>0){const Te=sg(Oe.width,Oe.height,A.format,A.type);for(const qe of A.layerUpdates){const pe=Oe.data.subarray(qe*Te/Oe.data.BYTES_PER_ELEMENT,(qe+1)*Te/Oe.data.BYTES_PER_ELEMENT);t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,de,0,0,qe,Oe.width,Oe.height,1,Ne,pe,0,0)}A.clearLayerUpdates()}else t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,de,0,0,0,Oe.width,Oe.height,Ee.depth,Ne,Oe.data,0,0)}else t.compressedTexImage3D(s.TEXTURE_2D_ARRAY,de,Je,Oe.width,Oe.height,Ee.depth,0,Oe.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else lt?X&&t.texSubImage3D(s.TEXTURE_2D_ARRAY,de,0,0,0,Oe.width,Oe.height,Ee.depth,Ne,pt,Oe.data):t.texImage3D(s.TEXTURE_2D_ARRAY,de,Je,Oe.width,Oe.height,Ee.depth,0,Ne,pt,Oe.data)}else{lt&&Tt&&t.texStorage2D(s.TEXTURE_2D,we,Je,st[0].width,st[0].height);for(let de=0,he=st.length;de<he;de++)Oe=st[de],A.format!==fi?Ne!==null?lt?X&&t.compressedTexSubImage2D(s.TEXTURE_2D,de,0,0,Oe.width,Oe.height,Ne,Oe.data):t.compressedTexImage2D(s.TEXTURE_2D,de,Je,Oe.width,Oe.height,0,Oe.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):lt?X&&t.texSubImage2D(s.TEXTURE_2D,de,0,0,Oe.width,Oe.height,Ne,pt,Oe.data):t.texImage2D(s.TEXTURE_2D,de,Je,Oe.width,Oe.height,0,Ne,pt,Oe.data)}else if(A.isDataArrayTexture)if(lt){if(Tt&&t.texStorage3D(s.TEXTURE_2D_ARRAY,we,Je,Ee.width,Ee.height,Ee.depth),X)if(A.layerUpdates.size>0){const de=sg(Ee.width,Ee.height,A.format,A.type);for(const he of A.layerUpdates){const Te=Ee.data.subarray(he*de/Ee.data.BYTES_PER_ELEMENT,(he+1)*de/Ee.data.BYTES_PER_ELEMENT);t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,he,Ee.width,Ee.height,1,Ne,pt,Te)}A.clearLayerUpdates()}else t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,0,Ee.width,Ee.height,Ee.depth,Ne,pt,Ee.data)}else t.texImage3D(s.TEXTURE_2D_ARRAY,0,Je,Ee.width,Ee.height,Ee.depth,0,Ne,pt,Ee.data);else if(A.isData3DTexture)lt?(Tt&&t.texStorage3D(s.TEXTURE_3D,we,Je,Ee.width,Ee.height,Ee.depth),X&&t.texSubImage3D(s.TEXTURE_3D,0,0,0,0,Ee.width,Ee.height,Ee.depth,Ne,pt,Ee.data)):t.texImage3D(s.TEXTURE_3D,0,Je,Ee.width,Ee.height,Ee.depth,0,Ne,pt,Ee.data);else if(A.isFramebufferTexture){if(Tt)if(lt)t.texStorage2D(s.TEXTURE_2D,we,Je,Ee.width,Ee.height);else{let de=Ee.width,he=Ee.height;for(let Te=0;Te<we;Te++)t.texImage2D(s.TEXTURE_2D,Te,Je,de,he,0,Ne,pt,null),de>>=1,he>>=1}}else if(st.length>0){if(lt&&Tt){const de=rt(st[0]);t.texStorage2D(s.TEXTURE_2D,we,Je,de.width,de.height)}for(let de=0,he=st.length;de<he;de++)Oe=st[de],lt?X&&t.texSubImage2D(s.TEXTURE_2D,de,0,0,Ne,pt,Oe):t.texImage2D(s.TEXTURE_2D,de,Je,Ne,pt,Oe);A.generateMipmaps=!1}else if(lt){if(Tt){const de=rt(Ee);t.texStorage2D(s.TEXTURE_2D,we,Je,de.width,de.height)}X&&t.texSubImage2D(s.TEXTURE_2D,0,0,0,Ne,pt,Ee)}else t.texImage2D(s.TEXTURE_2D,0,Je,Ne,pt,Ee);x(A)&&_(_e),je.__version=me.version,A.onUpdate&&A.onUpdate(A)}U.__version=A.version}function te(U,A,se){if(A.image.length!==6)return;const _e=ce(U,A),ye=A.source;t.bindTexture(s.TEXTURE_CUBE_MAP,U.__webglTexture,s.TEXTURE0+se);const me=r.get(ye);if(ye.version!==me.__version||_e===!0){t.activeTexture(s.TEXTURE0+se);const je=At.getPrimaries(At.workingColorSpace),be=A.colorSpace===vr?null:At.getPrimaries(A.colorSpace),De=A.colorSpace===vr||je===be?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,A.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,A.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,A.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,De);const ot=A.isCompressedTexture||A.image[0].isCompressedTexture,Ee=A.image[0]&&A.image[0].isDataTexture,Ne=[];for(let he=0;he<6;he++)!ot&&!Ee?Ne[he]=S(A.image[he],!0,o.maxCubemapSize):Ne[he]=Ee?A.image[he].image:A.image[he],Ne[he]=tt(A,Ne[he]);const pt=Ne[0],Je=u.convert(A.format,A.colorSpace),Oe=u.convert(A.type),st=P(A.internalFormat,Je,Oe,A.colorSpace),lt=A.isVideoTexture!==!0,Tt=me.__version===void 0||_e===!0,X=ye.dataReady;let we=L(A,pt);F(s.TEXTURE_CUBE_MAP,A);let de;if(ot){lt&&Tt&&t.texStorage2D(s.TEXTURE_CUBE_MAP,we,st,pt.width,pt.height);for(let he=0;he<6;he++){de=Ne[he].mipmaps;for(let Te=0;Te<de.length;Te++){const qe=de[Te];A.format!==fi?Je!==null?lt?X&&t.compressedTexSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te,0,0,qe.width,qe.height,Je,qe.data):t.compressedTexImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te,st,qe.width,qe.height,0,qe.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):lt?X&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te,0,0,qe.width,qe.height,Je,Oe,qe.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te,st,qe.width,qe.height,0,Je,Oe,qe.data)}}}else{if(de=A.mipmaps,lt&&Tt){de.length>0&&we++;const he=rt(Ne[0]);t.texStorage2D(s.TEXTURE_CUBE_MAP,we,st,he.width,he.height)}for(let he=0;he<6;he++)if(Ee){lt?X&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,0,0,0,Ne[he].width,Ne[he].height,Je,Oe,Ne[he].data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,0,st,Ne[he].width,Ne[he].height,0,Je,Oe,Ne[he].data);for(let Te=0;Te<de.length;Te++){const pe=de[Te].image[he].image;lt?X&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te+1,0,0,pe.width,pe.height,Je,Oe,pe.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te+1,st,pe.width,pe.height,0,Je,Oe,pe.data)}}else{lt?X&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,0,0,0,Je,Oe,Ne[he]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,0,st,Je,Oe,Ne[he]);for(let Te=0;Te<de.length;Te++){const qe=de[Te];lt?X&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te+1,0,0,Je,Oe,qe.image[he]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+he,Te+1,st,Je,Oe,qe.image[he])}}}x(A)&&_(s.TEXTURE_CUBE_MAP),me.__version=ye.version,A.onUpdate&&A.onUpdate(A)}U.__version=A.version}function fe(U,A,se,_e,ye,me){const je=u.convert(se.format,se.colorSpace),be=u.convert(se.type),De=P(se.internalFormat,je,be,se.colorSpace);if(!r.get(A).__hasExternalTextures){const Ee=Math.max(1,A.width>>me),Ne=Math.max(1,A.height>>me);ye===s.TEXTURE_3D||ye===s.TEXTURE_2D_ARRAY?t.texImage3D(ye,me,De,Ee,Ne,A.depth,0,je,be,null):t.texImage2D(ye,me,De,Ee,Ne,0,je,be,null)}t.bindFramebuffer(s.FRAMEBUFFER,U),We(A)?d.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,_e,ye,r.get(se).__webglTexture,0,yt(A)):(ye===s.TEXTURE_2D||ye>=s.TEXTURE_CUBE_MAP_POSITIVE_X&&ye<=s.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&s.framebufferTexture2D(s.FRAMEBUFFER,_e,ye,r.get(se).__webglTexture,me),t.bindFramebuffer(s.FRAMEBUFFER,null)}function xe(U,A,se){if(s.bindRenderbuffer(s.RENDERBUFFER,U),A.depthBuffer){const _e=A.depthTexture,ye=_e&&_e.isDepthTexture?_e.type:null,me=R(A.stencilBuffer,ye),je=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,be=yt(A);We(A)?d.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,be,me,A.width,A.height):se?s.renderbufferStorageMultisample(s.RENDERBUFFER,be,me,A.width,A.height):s.renderbufferStorage(s.RENDERBUFFER,me,A.width,A.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,je,s.RENDERBUFFER,U)}else{const _e=A.textures;for(let ye=0;ye<_e.length;ye++){const me=_e[ye],je=u.convert(me.format,me.colorSpace),be=u.convert(me.type),De=P(me.internalFormat,je,be,me.colorSpace),ot=yt(A);se&&We(A)===!1?s.renderbufferStorageMultisample(s.RENDERBUFFER,ot,De,A.width,A.height):We(A)?d.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,ot,De,A.width,A.height):s.renderbufferStorage(s.RENDERBUFFER,De,A.width,A.height)}}s.bindRenderbuffer(s.RENDERBUFFER,null)}function Me(U,A){if(A&&A.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(s.FRAMEBUFFER,U),!(A.depthTexture&&A.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!r.get(A.depthTexture).__webglTexture||A.depthTexture.image.width!==A.width||A.depthTexture.image.height!==A.height)&&(A.depthTexture.image.width=A.width,A.depthTexture.image.height=A.height,A.depthTexture.needsUpdate=!0),oe(A.depthTexture,0);const _e=r.get(A.depthTexture).__webglTexture,ye=yt(A);if(A.depthTexture.format===Qs)We(A)?d.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,_e,0,ye):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,_e,0);else if(A.depthTexture.format===ra)We(A)?d.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,_e,0,ye):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,_e,0);else throw new Error("Unknown depthTexture format")}function Le(U){const A=r.get(U),se=U.isWebGLCubeRenderTarget===!0;if(U.depthTexture&&!A.__autoAllocateDepthBuffer){if(se)throw new Error("target.depthTexture not supported in Cube render targets");Me(A.__webglFramebuffer,U)}else if(se){A.__webglDepthbuffer=[];for(let _e=0;_e<6;_e++)t.bindFramebuffer(s.FRAMEBUFFER,A.__webglFramebuffer[_e]),A.__webglDepthbuffer[_e]=s.createRenderbuffer(),xe(A.__webglDepthbuffer[_e],U,!1)}else t.bindFramebuffer(s.FRAMEBUFFER,A.__webglFramebuffer),A.__webglDepthbuffer=s.createRenderbuffer(),xe(A.__webglDepthbuffer,U,!1);t.bindFramebuffer(s.FRAMEBUFFER,null)}function ke(U,A,se){const _e=r.get(U);A!==void 0&&fe(_e.__webglFramebuffer,U,U.texture,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,0),se!==void 0&&Le(U)}function Ye(U){const A=U.texture,se=r.get(U),_e=r.get(A);U.addEventListener("dispose",O);const ye=U.textures,me=U.isWebGLCubeRenderTarget===!0,je=ye.length>1;if(je||(_e.__webglTexture===void 0&&(_e.__webglTexture=s.createTexture()),_e.__version=A.version,c.memory.textures++),me){se.__webglFramebuffer=[];for(let be=0;be<6;be++)if(A.mipmaps&&A.mipmaps.length>0){se.__webglFramebuffer[be]=[];for(let De=0;De<A.mipmaps.length;De++)se.__webglFramebuffer[be][De]=s.createFramebuffer()}else se.__webglFramebuffer[be]=s.createFramebuffer()}else{if(A.mipmaps&&A.mipmaps.length>0){se.__webglFramebuffer=[];for(let be=0;be<A.mipmaps.length;be++)se.__webglFramebuffer[be]=s.createFramebuffer()}else se.__webglFramebuffer=s.createFramebuffer();if(je)for(let be=0,De=ye.length;be<De;be++){const ot=r.get(ye[be]);ot.__webglTexture===void 0&&(ot.__webglTexture=s.createTexture(),c.memory.textures++)}if(U.samples>0&&We(U)===!1){se.__webglMultisampledFramebuffer=s.createFramebuffer(),se.__webglColorRenderbuffer=[],t.bindFramebuffer(s.FRAMEBUFFER,se.__webglMultisampledFramebuffer);for(let be=0;be<ye.length;be++){const De=ye[be];se.__webglColorRenderbuffer[be]=s.createRenderbuffer(),s.bindRenderbuffer(s.RENDERBUFFER,se.__webglColorRenderbuffer[be]);const ot=u.convert(De.format,De.colorSpace),Ee=u.convert(De.type),Ne=P(De.internalFormat,ot,Ee,De.colorSpace,U.isXRRenderTarget===!0),pt=yt(U);s.renderbufferStorageMultisample(s.RENDERBUFFER,pt,Ne,U.width,U.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+be,s.RENDERBUFFER,se.__webglColorRenderbuffer[be])}s.bindRenderbuffer(s.RENDERBUFFER,null),U.depthBuffer&&(se.__webglDepthRenderbuffer=s.createRenderbuffer(),xe(se.__webglDepthRenderbuffer,U,!0)),t.bindFramebuffer(s.FRAMEBUFFER,null)}}if(me){t.bindTexture(s.TEXTURE_CUBE_MAP,_e.__webglTexture),F(s.TEXTURE_CUBE_MAP,A);for(let be=0;be<6;be++)if(A.mipmaps&&A.mipmaps.length>0)for(let De=0;De<A.mipmaps.length;De++)fe(se.__webglFramebuffer[be][De],U,A,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+be,De);else fe(se.__webglFramebuffer[be],U,A,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+be,0);x(A)&&_(s.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(je){for(let be=0,De=ye.length;be<De;be++){const ot=ye[be],Ee=r.get(ot);t.bindTexture(s.TEXTURE_2D,Ee.__webglTexture),F(s.TEXTURE_2D,ot),fe(se.__webglFramebuffer,U,ot,s.COLOR_ATTACHMENT0+be,s.TEXTURE_2D,0),x(ot)&&_(s.TEXTURE_2D)}t.unbindTexture()}else{let be=s.TEXTURE_2D;if((U.isWebGL3DRenderTarget||U.isWebGLArrayRenderTarget)&&(be=U.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),t.bindTexture(be,_e.__webglTexture),F(be,A),A.mipmaps&&A.mipmaps.length>0)for(let De=0;De<A.mipmaps.length;De++)fe(se.__webglFramebuffer[De],U,A,s.COLOR_ATTACHMENT0,be,De);else fe(se.__webglFramebuffer,U,A,s.COLOR_ATTACHMENT0,be,0);x(A)&&_(be),t.unbindTexture()}U.depthBuffer&&Le(U)}function wt(U){const A=U.textures;for(let se=0,_e=A.length;se<_e;se++){const ye=A[se];if(x(ye)){const me=U.isWebGLCubeRenderTarget?s.TEXTURE_CUBE_MAP:s.TEXTURE_2D,je=r.get(ye).__webglTexture;t.bindTexture(me,je),_(me),t.unbindTexture()}}}const z=[],bt=[];function vt(U){if(U.samples>0){if(We(U)===!1){const A=U.textures,se=U.width,_e=U.height;let ye=s.COLOR_BUFFER_BIT;const me=U.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,je=r.get(U),be=A.length>1;if(be)for(let De=0;De<A.length;De++)t.bindFramebuffer(s.FRAMEBUFFER,je.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+De,s.RENDERBUFFER,null),t.bindFramebuffer(s.FRAMEBUFFER,je.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+De,s.TEXTURE_2D,null,0);t.bindFramebuffer(s.READ_FRAMEBUFFER,je.__webglMultisampledFramebuffer),t.bindFramebuffer(s.DRAW_FRAMEBUFFER,je.__webglFramebuffer);for(let De=0;De<A.length;De++){if(U.resolveDepthBuffer&&(U.depthBuffer&&(ye|=s.DEPTH_BUFFER_BIT),U.stencilBuffer&&U.resolveStencilBuffer&&(ye|=s.STENCIL_BUFFER_BIT)),be){s.framebufferRenderbuffer(s.READ_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.RENDERBUFFER,je.__webglColorRenderbuffer[De]);const ot=r.get(A[De]).__webglTexture;s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,ot,0)}s.blitFramebuffer(0,0,se,_e,0,0,se,_e,ye,s.NEAREST),h===!0&&(z.length=0,bt.length=0,z.push(s.COLOR_ATTACHMENT0+De),U.depthBuffer&&U.resolveDepthBuffer===!1&&(z.push(me),bt.push(me),s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,bt)),s.invalidateFramebuffer(s.READ_FRAMEBUFFER,z))}if(t.bindFramebuffer(s.READ_FRAMEBUFFER,null),t.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),be)for(let De=0;De<A.length;De++){t.bindFramebuffer(s.FRAMEBUFFER,je.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+De,s.RENDERBUFFER,je.__webglColorRenderbuffer[De]);const ot=r.get(A[De]).__webglTexture;t.bindFramebuffer(s.FRAMEBUFFER,je.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+De,s.TEXTURE_2D,ot,0)}t.bindFramebuffer(s.DRAW_FRAMEBUFFER,je.__webglMultisampledFramebuffer)}else if(U.depthBuffer&&U.resolveDepthBuffer===!1&&h){const A=U.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT;s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,[A])}}}function yt(U){return Math.min(o.maxSamples,U.samples)}function We(U){const A=r.get(U);return U.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&A.__useRenderToTexture!==!1}function Lt(U){const A=c.render.frame;g.get(U)!==A&&(g.set(U,A),U.update())}function tt(U,A){const se=U.colorSpace,_e=U.format,ye=U.type;return U.isCompressedTexture===!0||U.isVideoTexture===!0||se!==wr&&se!==vr&&(At.getTransfer(se)===Ot?(_e!==fi||ye!==Hi)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",se)),A}function rt(U){return typeof HTMLImageElement<"u"&&U instanceof HTMLImageElement?(m.width=U.naturalWidth||U.width,m.height=U.naturalHeight||U.height):typeof VideoFrame<"u"&&U instanceof VideoFrame?(m.width=U.displayWidth,m.height=U.displayHeight):(m.width=U.width,m.height=U.height),m}this.allocateTextureUnit=Y,this.resetTextureUnits=I,this.setTexture2D=oe,this.setTexture2DArray=ne,this.setTexture3D=B,this.setTextureCube=G,this.rebindTextures=ke,this.setupRenderTarget=Ye,this.updateRenderTargetMipmap=wt,this.updateMultisampleRenderTarget=vt,this.setupDepthRenderbuffer=Le,this.setupFrameBufferTexture=fe,this.useMultisampledRTT=We}function KE(s,e){function t(r,o=vr){let u;const c=At.getTransfer(o);if(r===Hi)return s.UNSIGNED_BYTE;if(r===_d)return s.UNSIGNED_SHORT_4_4_4_4;if(r===vd)return s.UNSIGNED_SHORT_5_5_5_1;if(r===bg)return s.UNSIGNED_INT_5_9_9_9_REV;if(r===Cg)return s.BYTE;if(r===Rg)return s.SHORT;if(r===Qa)return s.UNSIGNED_SHORT;if(r===gd)return s.INT;if(r===Jr)return s.UNSIGNED_INT;if(r===Bi)return s.FLOAT;if(r===eo)return s.HALF_FLOAT;if(r===Pg)return s.ALPHA;if(r===Lg)return s.RGB;if(r===fi)return s.RGBA;if(r===Ng)return s.LUMINANCE;if(r===Dg)return s.LUMINANCE_ALPHA;if(r===Qs)return s.DEPTH_COMPONENT;if(r===ra)return s.DEPTH_STENCIL;if(r===Ig)return s.RED;if(r===xd)return s.RED_INTEGER;if(r===Ug)return s.RG;if(r===yd)return s.RG_INTEGER;if(r===Sd)return s.RGBA_INTEGER;if(r===zl||r===Hl||r===Vl||r===Gl)if(c===Ot)if(u=e.get("WEBGL_compressed_texture_s3tc_srgb"),u!==null){if(r===zl)return u.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(r===Hl)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(r===Vl)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(r===Gl)return u.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(u=e.get("WEBGL_compressed_texture_s3tc"),u!==null){if(r===zl)return u.COMPRESSED_RGB_S3TC_DXT1_EXT;if(r===Hl)return u.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(r===Vl)return u.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(r===Gl)return u.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(r===Of||r===kf||r===Bf||r===zf)if(u=e.get("WEBGL_compressed_texture_pvrtc"),u!==null){if(r===Of)return u.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(r===kf)return u.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(r===Bf)return u.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(r===zf)return u.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(r===Hf||r===Vf||r===Gf)if(u=e.get("WEBGL_compressed_texture_etc"),u!==null){if(r===Hf||r===Vf)return c===Ot?u.COMPRESSED_SRGB8_ETC2:u.COMPRESSED_RGB8_ETC2;if(r===Gf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:u.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(r===Wf||r===Xf||r===jf||r===Yf||r===qf||r===$f||r===Kf||r===Zf||r===Qf||r===Jf||r===ed||r===td||r===nd||r===id)if(u=e.get("WEBGL_compressed_texture_astc"),u!==null){if(r===Wf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:u.COMPRESSED_RGBA_ASTC_4x4_KHR;if(r===Xf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:u.COMPRESSED_RGBA_ASTC_5x4_KHR;if(r===jf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:u.COMPRESSED_RGBA_ASTC_5x5_KHR;if(r===Yf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:u.COMPRESSED_RGBA_ASTC_6x5_KHR;if(r===qf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:u.COMPRESSED_RGBA_ASTC_6x6_KHR;if(r===$f)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:u.COMPRESSED_RGBA_ASTC_8x5_KHR;if(r===Kf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:u.COMPRESSED_RGBA_ASTC_8x6_KHR;if(r===Zf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:u.COMPRESSED_RGBA_ASTC_8x8_KHR;if(r===Qf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:u.COMPRESSED_RGBA_ASTC_10x5_KHR;if(r===Jf)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:u.COMPRESSED_RGBA_ASTC_10x6_KHR;if(r===ed)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:u.COMPRESSED_RGBA_ASTC_10x8_KHR;if(r===td)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:u.COMPRESSED_RGBA_ASTC_10x10_KHR;if(r===nd)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:u.COMPRESSED_RGBA_ASTC_12x10_KHR;if(r===id)return c===Ot?u.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:u.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(r===Wl||r===rd||r===sd)if(u=e.get("EXT_texture_compression_bptc"),u!==null){if(r===Wl)return c===Ot?u.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:u.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(r===rd)return u.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(r===sd)return u.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(r===Fg||r===ad||r===od||r===ld)if(u=e.get("EXT_texture_compression_rgtc"),u!==null){if(r===Wl)return u.COMPRESSED_RED_RGTC1_EXT;if(r===ad)return u.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(r===od)return u.COMPRESSED_RED_GREEN_RGTC2_EXT;if(r===ld)return u.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return r===ia?s.UNSIGNED_INT_24_8:s[r]!==void 0?s[r]:null}return{convert:t}}class ZE extends $n{constructor(e=[]){super(),this.isArrayCamera=!0,this.cameras=e}}class Zr extends Qt{constructor(){super(),this.isGroup=!0,this.type="Group"}}const QE={type:"move"};class xf{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Zr,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Zr,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new J,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new J),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Zr,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new J,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new J),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){const t=this._hand;if(t)for(const r of e.hand.values())this._getHandJoint(t,r)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,r){let o=null,u=null,c=null;const d=this._targetRay,h=this._grip,m=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(m&&e.hand){c=!0;for(const S of e.hand.values()){const x=t.getJointPose(S,r),_=this._getHandJoint(m,S);x!==null&&(_.matrix.fromArray(x.transform.matrix),_.matrix.decompose(_.position,_.rotation,_.scale),_.matrixWorldNeedsUpdate=!0,_.jointRadius=x.radius),_.visible=x!==null}const g=m.joints["index-finger-tip"],y=m.joints["thumb-tip"],v=g.position.distanceTo(y.position),M=.02,T=.005;m.inputState.pinching&&v>M+T?(m.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!m.inputState.pinching&&v<=M-T&&(m.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else h!==null&&e.gripSpace&&(u=t.getPose(e.gripSpace,r),u!==null&&(h.matrix.fromArray(u.transform.matrix),h.matrix.decompose(h.position,h.rotation,h.scale),h.matrixWorldNeedsUpdate=!0,u.linearVelocity?(h.hasLinearVelocity=!0,h.linearVelocity.copy(u.linearVelocity)):h.hasLinearVelocity=!1,u.angularVelocity?(h.hasAngularVelocity=!0,h.angularVelocity.copy(u.angularVelocity)):h.hasAngularVelocity=!1));d!==null&&(o=t.getPose(e.targetRaySpace,r),o===null&&u!==null&&(o=u),o!==null&&(d.matrix.fromArray(o.transform.matrix),d.matrix.decompose(d.position,d.rotation,d.scale),d.matrixWorldNeedsUpdate=!0,o.linearVelocity?(d.hasLinearVelocity=!0,d.linearVelocity.copy(o.linearVelocity)):d.hasLinearVelocity=!1,o.angularVelocity?(d.hasAngularVelocity=!0,d.angularVelocity.copy(o.angularVelocity)):d.hasAngularVelocity=!1,this.dispatchEvent(QE)))}return d!==null&&(d.visible=o!==null),h!==null&&(h.visible=u!==null),m!==null&&(m.visible=c!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){const r=new Zr;r.matrixAutoUpdate=!1,r.visible=!1,e.joints[t.jointName]=r,e.add(r)}return e.joints[t.jointName]}}const JE=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,ew=`
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

}`;class tw{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t,r){if(this.texture===null){const o=new En,u=e.properties.get(o);u.__webglTexture=t.texture,(t.depthNear!=r.depthNear||t.depthFar!=r.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=o}}getMesh(e){if(this.texture!==null&&this.mesh===null){const t=e.cameras[0].viewport,r=new Er({vertexShader:JE,fragmentShader:ew,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new zn(new ru(20,20),r)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}}class nw extends aa{constructor(e,t){super();const r=this;let o=null,u=1,c=null,d="local-floor",h=1,m=null,g=null,y=null,v=null,M=null,T=null;const S=new tw,x=t.getContextAttributes();let _=null,P=null;const R=[],L=[],$=new ft;let O=null;const D=new $n;D.layers.enable(1),D.viewport=new Zt;const j=new $n;j.layers.enable(2),j.viewport=new Zt;const b=[D,j],w=new ZE;w.layers.enable(1),w.layers.enable(2);let I=null,Y=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(te){let fe=R[te];return fe===void 0&&(fe=new xf,R[te]=fe),fe.getTargetRaySpace()},this.getControllerGrip=function(te){let fe=R[te];return fe===void 0&&(fe=new xf,R[te]=fe),fe.getGripSpace()},this.getHand=function(te){let fe=R[te];return fe===void 0&&(fe=new xf,R[te]=fe),fe.getHandSpace()};function K(te){const fe=L.indexOf(te.inputSource);if(fe===-1)return;const xe=R[fe];xe!==void 0&&(xe.update(te.inputSource,te.frame,m||c),xe.dispatchEvent({type:te.type,data:te.inputSource}))}function oe(){o.removeEventListener("select",K),o.removeEventListener("selectstart",K),o.removeEventListener("selectend",K),o.removeEventListener("squeeze",K),o.removeEventListener("squeezestart",K),o.removeEventListener("squeezeend",K),o.removeEventListener("end",oe),o.removeEventListener("inputsourceschange",ne);for(let te=0;te<R.length;te++){const fe=L[te];fe!==null&&(L[te]=null,R[te].disconnect(fe))}I=null,Y=null,S.reset(),e.setRenderTarget(_),M=null,v=null,y=null,o=null,P=null,Ie.stop(),r.isPresenting=!1,e.setPixelRatio(O),e.setSize($.width,$.height,!1),r.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(te){u=te,r.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(te){d=te,r.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return m||c},this.setReferenceSpace=function(te){m=te},this.getBaseLayer=function(){return v!==null?v:M},this.getBinding=function(){return y},this.getFrame=function(){return T},this.getSession=function(){return o},this.setSession=async function(te){if(o=te,o!==null){if(_=e.getRenderTarget(),o.addEventListener("select",K),o.addEventListener("selectstart",K),o.addEventListener("selectend",K),o.addEventListener("squeeze",K),o.addEventListener("squeezestart",K),o.addEventListener("squeezeend",K),o.addEventListener("end",oe),o.addEventListener("inputsourceschange",ne),x.xrCompatible!==!0&&await t.makeXRCompatible(),O=e.getPixelRatio(),e.getSize($),o.renderState.layers===void 0){const fe={antialias:x.antialias,alpha:!0,depth:x.depth,stencil:x.stencil,framebufferScaleFactor:u};M=new XRWebGLLayer(o,t,fe),o.updateRenderState({baseLayer:M}),e.setPixelRatio(1),e.setSize(M.framebufferWidth,M.framebufferHeight,!1),P=new es(M.framebufferWidth,M.framebufferHeight,{format:fi,type:Hi,colorSpace:e.outputColorSpace,stencilBuffer:x.stencil})}else{let fe=null,xe=null,Me=null;x.depth&&(Me=x.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,fe=x.stencil?ra:Qs,xe=x.stencil?ia:Jr);const Le={colorFormat:t.RGBA8,depthFormat:Me,scaleFactor:u};y=new XRWebGLBinding(o,t),v=y.createProjectionLayer(Le),o.updateRenderState({layers:[v]}),e.setPixelRatio(1),e.setSize(v.textureWidth,v.textureHeight,!1),P=new es(v.textureWidth,v.textureHeight,{format:fi,type:Hi,depthTexture:new Zg(v.textureWidth,v.textureHeight,xe,void 0,void 0,void 0,void 0,void 0,void 0,fe),stencilBuffer:x.stencil,colorSpace:e.outputColorSpace,samples:x.antialias?4:0,resolveDepthBuffer:v.ignoreDepthValues===!1})}P.isXRRenderTarget=!0,this.setFoveation(h),m=null,c=await o.requestReferenceSpace(d),Ie.setContext(o),Ie.start(),r.isPresenting=!0,r.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(o!==null)return o.environmentBlendMode},this.getDepthTexture=function(){return S.getDepthTexture()};function ne(te){for(let fe=0;fe<te.removed.length;fe++){const xe=te.removed[fe],Me=L.indexOf(xe);Me>=0&&(L[Me]=null,R[Me].disconnect(xe))}for(let fe=0;fe<te.added.length;fe++){const xe=te.added[fe];let Me=L.indexOf(xe);if(Me===-1){for(let ke=0;ke<R.length;ke++)if(ke>=L.length){L.push(xe),Me=ke;break}else if(L[ke]===null){L[ke]=xe,Me=ke;break}if(Me===-1)break}const Le=R[Me];Le&&Le.connect(xe)}}const B=new J,G=new J;function k(te,fe,xe){B.setFromMatrixPosition(fe.matrixWorld),G.setFromMatrixPosition(xe.matrixWorld);const Me=B.distanceTo(G),Le=fe.projectionMatrix.elements,ke=xe.projectionMatrix.elements,Ye=Le[14]/(Le[10]-1),wt=Le[14]/(Le[10]+1),z=(Le[9]+1)/Le[5],bt=(Le[9]-1)/Le[5],vt=(Le[8]-1)/Le[0],yt=(ke[8]+1)/ke[0],We=Ye*vt,Lt=Ye*yt,tt=Me/(-vt+yt),rt=tt*-vt;fe.matrixWorld.decompose(te.position,te.quaternion,te.scale),te.translateX(rt),te.translateZ(tt),te.matrixWorld.compose(te.position,te.quaternion,te.scale),te.matrixWorldInverse.copy(te.matrixWorld).invert();const U=Ye+tt,A=wt+tt,se=We-rt,_e=Lt+(Me-rt),ye=z*wt/A*U,me=bt*wt/A*U;te.projectionMatrix.makePerspective(se,_e,ye,me,U,A),te.projectionMatrixInverse.copy(te.projectionMatrix).invert()}function ue(te,fe){fe===null?te.matrixWorld.copy(te.matrix):te.matrixWorld.multiplyMatrices(fe.matrixWorld,te.matrix),te.matrixWorldInverse.copy(te.matrixWorld).invert()}this.updateCamera=function(te){if(o===null)return;S.texture!==null&&(te.near=S.depthNear,te.far=S.depthFar),w.near=j.near=D.near=te.near,w.far=j.far=D.far=te.far,(I!==w.near||Y!==w.far)&&(o.updateRenderState({depthNear:w.near,depthFar:w.far}),I=w.near,Y=w.far,D.near=I,D.far=Y,j.near=I,j.far=Y,D.updateProjectionMatrix(),j.updateProjectionMatrix(),te.updateProjectionMatrix());const fe=te.parent,xe=w.cameras;ue(w,fe);for(let Me=0;Me<xe.length;Me++)ue(xe[Me],fe);xe.length===2?k(w,D,j):w.projectionMatrix.copy(D.projectionMatrix),le(te,w,fe)};function le(te,fe,xe){xe===null?te.matrix.copy(fe.matrixWorld):(te.matrix.copy(xe.matrixWorld),te.matrix.invert(),te.matrix.multiply(fe.matrixWorld)),te.matrix.decompose(te.position,te.quaternion,te.scale),te.updateMatrixWorld(!0),te.projectionMatrix.copy(fe.projectionMatrix),te.projectionMatrixInverse.copy(fe.projectionMatrixInverse),te.isPerspectiveCamera&&(te.fov=cd*2*Math.atan(1/te.projectionMatrix.elements[5]),te.zoom=1)}this.getCamera=function(){return w},this.getFoveation=function(){if(!(v===null&&M===null))return h},this.setFoveation=function(te){h=te,v!==null&&(v.fixedFoveation=te),M!==null&&M.fixedFoveation!==void 0&&(M.fixedFoveation=te)},this.hasDepthSensing=function(){return S.texture!==null},this.getDepthSensingMesh=function(){return S.getMesh(w)};let F=null;function ce(te,fe){if(g=fe.getViewerPose(m||c),T=fe,g!==null){const xe=g.views;M!==null&&(e.setRenderTargetFramebuffer(P,M.framebuffer),e.setRenderTarget(P));let Me=!1;xe.length!==w.cameras.length&&(w.cameras.length=0,Me=!0);for(let ke=0;ke<xe.length;ke++){const Ye=xe[ke];let wt=null;if(M!==null)wt=M.getViewport(Ye);else{const bt=y.getViewSubImage(v,Ye);wt=bt.viewport,ke===0&&(e.setRenderTargetTextures(P,bt.colorTexture,v.ignoreDepthValues?void 0:bt.depthStencilTexture),e.setRenderTarget(P))}let z=b[ke];z===void 0&&(z=new $n,z.layers.enable(ke),z.viewport=new Zt,b[ke]=z),z.matrix.fromArray(Ye.transform.matrix),z.matrix.decompose(z.position,z.quaternion,z.scale),z.projectionMatrix.fromArray(Ye.projectionMatrix),z.projectionMatrixInverse.copy(z.projectionMatrix).invert(),z.viewport.set(wt.x,wt.y,wt.width,wt.height),ke===0&&(w.matrix.copy(z.matrix),w.matrix.decompose(w.position,w.quaternion,w.scale)),Me===!0&&w.cameras.push(z)}const Le=o.enabledFeatures;if(Le&&Le.includes("depth-sensing")){const ke=y.getDepthInformation(xe[0]);ke&&ke.isValid&&ke.texture&&S.init(e,ke,o.renderState)}}for(let xe=0;xe<R.length;xe++){const Me=L[xe],Le=R[xe];Me!==null&&Le!==void 0&&Le.update(Me,fe,m||c)}F&&F(te,fe),fe.detectedPlanes&&r.dispatchEvent({type:"planesdetected",data:fe}),T=null}const Ie=new Kg;Ie.setAnimationLoop(ce),this.setAnimationLoop=function(te){F=te},this.dispose=function(){}}}const Wr=new yi,iw=new Vt;function rw(s,e){function t(x,_){x.matrixAutoUpdate===!0&&x.updateMatrix(),_.value.copy(x.matrix)}function r(x,_){_.color.getRGB(x.fogColor.value,Yg(s)),_.isFog?(x.fogNear.value=_.near,x.fogFar.value=_.far):_.isFogExp2&&(x.fogDensity.value=_.density)}function o(x,_,P,R,L){_.isMeshBasicMaterial||_.isMeshLambertMaterial?u(x,_):_.isMeshToonMaterial?(u(x,_),y(x,_)):_.isMeshPhongMaterial?(u(x,_),g(x,_)):_.isMeshStandardMaterial?(u(x,_),v(x,_),_.isMeshPhysicalMaterial&&M(x,_,L)):_.isMeshMatcapMaterial?(u(x,_),T(x,_)):_.isMeshDepthMaterial?u(x,_):_.isMeshDistanceMaterial?(u(x,_),S(x,_)):_.isMeshNormalMaterial?u(x,_):_.isLineBasicMaterial?(c(x,_),_.isLineDashedMaterial&&d(x,_)):_.isPointsMaterial?h(x,_,P,R):_.isSpriteMaterial?m(x,_):_.isShadowMaterial?(x.color.value.copy(_.color),x.opacity.value=_.opacity):_.isShaderMaterial&&(_.uniformsNeedUpdate=!1)}function u(x,_){x.opacity.value=_.opacity,_.color&&x.diffuse.value.copy(_.color),_.emissive&&x.emissive.value.copy(_.emissive).multiplyScalar(_.emissiveIntensity),_.map&&(x.map.value=_.map,t(_.map,x.mapTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.bumpMap&&(x.bumpMap.value=_.bumpMap,t(_.bumpMap,x.bumpMapTransform),x.bumpScale.value=_.bumpScale,_.side===Ln&&(x.bumpScale.value*=-1)),_.normalMap&&(x.normalMap.value=_.normalMap,t(_.normalMap,x.normalMapTransform),x.normalScale.value.copy(_.normalScale),_.side===Ln&&x.normalScale.value.negate()),_.displacementMap&&(x.displacementMap.value=_.displacementMap,t(_.displacementMap,x.displacementMapTransform),x.displacementScale.value=_.displacementScale,x.displacementBias.value=_.displacementBias),_.emissiveMap&&(x.emissiveMap.value=_.emissiveMap,t(_.emissiveMap,x.emissiveMapTransform)),_.specularMap&&(x.specularMap.value=_.specularMap,t(_.specularMap,x.specularMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest);const P=e.get(_),R=P.envMap,L=P.envMapRotation;R&&(x.envMap.value=R,Wr.copy(L),Wr.x*=-1,Wr.y*=-1,Wr.z*=-1,R.isCubeTexture&&R.isRenderTargetTexture===!1&&(Wr.y*=-1,Wr.z*=-1),x.envMapRotation.value.setFromMatrix4(iw.makeRotationFromEuler(Wr)),x.flipEnvMap.value=R.isCubeTexture&&R.isRenderTargetTexture===!1?-1:1,x.reflectivity.value=_.reflectivity,x.ior.value=_.ior,x.refractionRatio.value=_.refractionRatio),_.lightMap&&(x.lightMap.value=_.lightMap,x.lightMapIntensity.value=_.lightMapIntensity,t(_.lightMap,x.lightMapTransform)),_.aoMap&&(x.aoMap.value=_.aoMap,x.aoMapIntensity.value=_.aoMapIntensity,t(_.aoMap,x.aoMapTransform))}function c(x,_){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,_.map&&(x.map.value=_.map,t(_.map,x.mapTransform))}function d(x,_){x.dashSize.value=_.dashSize,x.totalSize.value=_.dashSize+_.gapSize,x.scale.value=_.scale}function h(x,_,P,R){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,x.size.value=_.size*P,x.scale.value=R*.5,_.map&&(x.map.value=_.map,t(_.map,x.uvTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest)}function m(x,_){x.diffuse.value.copy(_.color),x.opacity.value=_.opacity,x.rotation.value=_.rotation,_.map&&(x.map.value=_.map,t(_.map,x.mapTransform)),_.alphaMap&&(x.alphaMap.value=_.alphaMap,t(_.alphaMap,x.alphaMapTransform)),_.alphaTest>0&&(x.alphaTest.value=_.alphaTest)}function g(x,_){x.specular.value.copy(_.specular),x.shininess.value=Math.max(_.shininess,1e-4)}function y(x,_){_.gradientMap&&(x.gradientMap.value=_.gradientMap)}function v(x,_){x.metalness.value=_.metalness,_.metalnessMap&&(x.metalnessMap.value=_.metalnessMap,t(_.metalnessMap,x.metalnessMapTransform)),x.roughness.value=_.roughness,_.roughnessMap&&(x.roughnessMap.value=_.roughnessMap,t(_.roughnessMap,x.roughnessMapTransform)),_.envMap&&(x.envMapIntensity.value=_.envMapIntensity)}function M(x,_,P){x.ior.value=_.ior,_.sheen>0&&(x.sheenColor.value.copy(_.sheenColor).multiplyScalar(_.sheen),x.sheenRoughness.value=_.sheenRoughness,_.sheenColorMap&&(x.sheenColorMap.value=_.sheenColorMap,t(_.sheenColorMap,x.sheenColorMapTransform)),_.sheenRoughnessMap&&(x.sheenRoughnessMap.value=_.sheenRoughnessMap,t(_.sheenRoughnessMap,x.sheenRoughnessMapTransform))),_.clearcoat>0&&(x.clearcoat.value=_.clearcoat,x.clearcoatRoughness.value=_.clearcoatRoughness,_.clearcoatMap&&(x.clearcoatMap.value=_.clearcoatMap,t(_.clearcoatMap,x.clearcoatMapTransform)),_.clearcoatRoughnessMap&&(x.clearcoatRoughnessMap.value=_.clearcoatRoughnessMap,t(_.clearcoatRoughnessMap,x.clearcoatRoughnessMapTransform)),_.clearcoatNormalMap&&(x.clearcoatNormalMap.value=_.clearcoatNormalMap,t(_.clearcoatNormalMap,x.clearcoatNormalMapTransform),x.clearcoatNormalScale.value.copy(_.clearcoatNormalScale),_.side===Ln&&x.clearcoatNormalScale.value.negate())),_.dispersion>0&&(x.dispersion.value=_.dispersion),_.iridescence>0&&(x.iridescence.value=_.iridescence,x.iridescenceIOR.value=_.iridescenceIOR,x.iridescenceThicknessMinimum.value=_.iridescenceThicknessRange[0],x.iridescenceThicknessMaximum.value=_.iridescenceThicknessRange[1],_.iridescenceMap&&(x.iridescenceMap.value=_.iridescenceMap,t(_.iridescenceMap,x.iridescenceMapTransform)),_.iridescenceThicknessMap&&(x.iridescenceThicknessMap.value=_.iridescenceThicknessMap,t(_.iridescenceThicknessMap,x.iridescenceThicknessMapTransform))),_.transmission>0&&(x.transmission.value=_.transmission,x.transmissionSamplerMap.value=P.texture,x.transmissionSamplerSize.value.set(P.width,P.height),_.transmissionMap&&(x.transmissionMap.value=_.transmissionMap,t(_.transmissionMap,x.transmissionMapTransform)),x.thickness.value=_.thickness,_.thicknessMap&&(x.thicknessMap.value=_.thicknessMap,t(_.thicknessMap,x.thicknessMapTransform)),x.attenuationDistance.value=_.attenuationDistance,x.attenuationColor.value.copy(_.attenuationColor)),_.anisotropy>0&&(x.anisotropyVector.value.set(_.anisotropy*Math.cos(_.anisotropyRotation),_.anisotropy*Math.sin(_.anisotropyRotation)),_.anisotropyMap&&(x.anisotropyMap.value=_.anisotropyMap,t(_.anisotropyMap,x.anisotropyMapTransform))),x.specularIntensity.value=_.specularIntensity,x.specularColor.value.copy(_.specularColor),_.specularColorMap&&(x.specularColorMap.value=_.specularColorMap,t(_.specularColorMap,x.specularColorMapTransform)),_.specularIntensityMap&&(x.specularIntensityMap.value=_.specularIntensityMap,t(_.specularIntensityMap,x.specularIntensityMapTransform))}function T(x,_){_.matcap&&(x.matcap.value=_.matcap)}function S(x,_){const P=e.get(_).light;x.referencePosition.value.setFromMatrixPosition(P.matrixWorld),x.nearDistance.value=P.shadow.camera.near,x.farDistance.value=P.shadow.camera.far}return{refreshFogUniforms:r,refreshMaterialUniforms:o}}function sw(s,e,t,r){let o={},u={},c=[];const d=s.getParameter(s.MAX_UNIFORM_BUFFER_BINDINGS);function h(P,R){const L=R.program;r.uniformBlockBinding(P,L)}function m(P,R){let L=o[P.id];L===void 0&&(T(P),L=g(P),o[P.id]=L,P.addEventListener("dispose",x));const $=R.program;r.updateUBOMapping(P,$);const O=e.render.frame;u[P.id]!==O&&(v(P),u[P.id]=O)}function g(P){const R=y();P.__bindingPointIndex=R;const L=s.createBuffer(),$=P.__size,O=P.usage;return s.bindBuffer(s.UNIFORM_BUFFER,L),s.bufferData(s.UNIFORM_BUFFER,$,O),s.bindBuffer(s.UNIFORM_BUFFER,null),s.bindBufferBase(s.UNIFORM_BUFFER,R,L),L}function y(){for(let P=0;P<d;P++)if(c.indexOf(P)===-1)return c.push(P),P;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function v(P){const R=o[P.id],L=P.uniforms,$=P.__cache;s.bindBuffer(s.UNIFORM_BUFFER,R);for(let O=0,D=L.length;O<D;O++){const j=Array.isArray(L[O])?L[O]:[L[O]];for(let b=0,w=j.length;b<w;b++){const I=j[b];if(M(I,O,b,$)===!0){const Y=I.__offset,K=Array.isArray(I.value)?I.value:[I.value];let oe=0;for(let ne=0;ne<K.length;ne++){const B=K[ne],G=S(B);typeof B=="number"||typeof B=="boolean"?(I.__data[0]=B,s.bufferSubData(s.UNIFORM_BUFFER,Y+oe,I.__data)):B.isMatrix3?(I.__data[0]=B.elements[0],I.__data[1]=B.elements[1],I.__data[2]=B.elements[2],I.__data[3]=0,I.__data[4]=B.elements[3],I.__data[5]=B.elements[4],I.__data[6]=B.elements[5],I.__data[7]=0,I.__data[8]=B.elements[6],I.__data[9]=B.elements[7],I.__data[10]=B.elements[8],I.__data[11]=0):(B.toArray(I.__data,oe),oe+=G.storage/Float32Array.BYTES_PER_ELEMENT)}s.bufferSubData(s.UNIFORM_BUFFER,Y,I.__data)}}}s.bindBuffer(s.UNIFORM_BUFFER,null)}function M(P,R,L,$){const O=P.value,D=R+"_"+L;if($[D]===void 0)return typeof O=="number"||typeof O=="boolean"?$[D]=O:$[D]=O.clone(),!0;{const j=$[D];if(typeof O=="number"||typeof O=="boolean"){if(j!==O)return $[D]=O,!0}else if(j.equals(O)===!1)return j.copy(O),!0}return!1}function T(P){const R=P.uniforms;let L=0;const $=16;for(let D=0,j=R.length;D<j;D++){const b=Array.isArray(R[D])?R[D]:[R[D]];for(let w=0,I=b.length;w<I;w++){const Y=b[w],K=Array.isArray(Y.value)?Y.value:[Y.value];for(let oe=0,ne=K.length;oe<ne;oe++){const B=K[oe],G=S(B),k=L%$,ue=k%G.boundary,le=k+ue;L+=ue,le!==0&&$-le<G.storage&&(L+=$-le),Y.__data=new Float32Array(G.storage/Float32Array.BYTES_PER_ELEMENT),Y.__offset=L,L+=G.storage}}}const O=L%$;return O>0&&(L+=$-O),P.__size=L,P.__cache={},this}function S(P){const R={boundary:0,storage:0};return typeof P=="number"||typeof P=="boolean"?(R.boundary=4,R.storage=4):P.isVector2?(R.boundary=8,R.storage=8):P.isVector3||P.isColor?(R.boundary=16,R.storage=12):P.isVector4?(R.boundary=16,R.storage=16):P.isMatrix3?(R.boundary=48,R.storage=48):P.isMatrix4?(R.boundary=64,R.storage=64):P.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",P),R}function x(P){const R=P.target;R.removeEventListener("dispose",x);const L=c.indexOf(R.__bindingPointIndex);c.splice(L,1),s.deleteBuffer(o[R.id]),delete o[R.id],delete u[R.id]}function _(){for(const P in o)s.deleteBuffer(o[P]);c=[],o={},u={}}return{bind:h,update:m,dispose:_}}class aw{constructor(e={}){const{canvas:t=K0(),context:r=null,depth:o=!0,stencil:u=!1,alpha:c=!1,antialias:d=!1,premultipliedAlpha:h=!0,preserveDrawingBuffer:m=!1,powerPreference:g="default",failIfMajorPerformanceCaveat:y=!1}=e;this.isWebGLRenderer=!0;let v;if(r!==null){if(typeof WebGLRenderingContext<"u"&&r instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");v=r.getContextAttributes().alpha}else v=c;const M=new Uint32Array(4),T=new Int32Array(4);let S=null,x=null;const _=[],P=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=li,this.toneMapping=yr,this.toneMappingExposure=1;const R=this;let L=!1,$=0,O=0,D=null,j=-1,b=null;const w=new Zt,I=new Zt;let Y=null;const K=new _t(0);let oe=0,ne=t.width,B=t.height,G=1,k=null,ue=null;const le=new Zt(0,0,ne,B),F=new Zt(0,0,ne,B);let ce=!1;const Ie=new wd;let te=!1,fe=!1;const xe=new Vt,Me=new J,Le=new Zt,ke={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let Ye=!1;function wt(){return D===null?G:1}let z=r;function bt(C,q){return t.getContext(C,q)}try{const C={alpha:!0,depth:o,stencil:u,antialias:d,premultipliedAlpha:h,preserveDrawingBuffer:m,powerPreference:g,failIfMajorPerformanceCaveat:y};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${md}`),t.addEventListener("webglcontextlost",de,!1),t.addEventListener("webglcontextrestored",he,!1),t.addEventListener("webglcontextcreationerror",Te,!1),z===null){const q="webgl2";if(z=bt(q,C),z===null)throw bt(q)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(C){throw console.error("THREE.WebGLRenderer: "+C.message),C}let vt,yt,We,Lt,tt,rt,U,A,se,_e,ye,me,je,be,De,ot,Ee,Ne,pt,Je,Oe,st,lt,Tt;function X(){vt=new dM(z),vt.init(),st=new KE(z,vt),yt=new aM(z,vt,e,st),We=new YE(z),Lt=new mM(z),tt=new DE,rt=new $E(z,vt,We,tt,yt,st,Lt),U=new lM(R),A=new fM(R),se=new Mx(z),lt=new rM(z,se),_e=new hM(z,se,Lt,lt),ye=new _M(z,_e,se,Lt),pt=new gM(z,yt,rt),ot=new oM(tt),me=new NE(R,U,A,vt,yt,lt,ot),je=new rw(R,tt),be=new UE,De=new HE(vt),Ne=new iM(R,U,A,We,ye,v,h),Ee=new jE(R,ye,yt),Tt=new sw(z,Lt,yt,We),Je=new sM(z,vt,Lt),Oe=new pM(z,vt,Lt),Lt.programs=me.programs,R.capabilities=yt,R.extensions=vt,R.properties=tt,R.renderLists=be,R.shadowMap=Ee,R.state=We,R.info=Lt}X();const we=new nw(R,z);this.xr=we,this.getContext=function(){return z},this.getContextAttributes=function(){return z.getContextAttributes()},this.forceContextLoss=function(){const C=vt.get("WEBGL_lose_context");C&&C.loseContext()},this.forceContextRestore=function(){const C=vt.get("WEBGL_lose_context");C&&C.restoreContext()},this.getPixelRatio=function(){return G},this.setPixelRatio=function(C){C!==void 0&&(G=C,this.setSize(ne,B,!1))},this.getSize=function(C){return C.set(ne,B)},this.setSize=function(C,q,ie=!0){if(we.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}ne=C,B=q,t.width=Math.floor(C*G),t.height=Math.floor(q*G),ie===!0&&(t.style.width=C+"px",t.style.height=q+"px"),this.setViewport(0,0,C,q)},this.getDrawingBufferSize=function(C){return C.set(ne*G,B*G).floor()},this.setDrawingBufferSize=function(C,q,ie){ne=C,B=q,G=ie,t.width=Math.floor(C*ie),t.height=Math.floor(q*ie),this.setViewport(0,0,C,q)},this.getCurrentViewport=function(C){return C.copy(w)},this.getViewport=function(C){return C.copy(le)},this.setViewport=function(C,q,ie,ae){C.isVector4?le.set(C.x,C.y,C.z,C.w):le.set(C,q,ie,ae),We.viewport(w.copy(le).multiplyScalar(G).round())},this.getScissor=function(C){return C.copy(F)},this.setScissor=function(C,q,ie,ae){C.isVector4?F.set(C.x,C.y,C.z,C.w):F.set(C,q,ie,ae),We.scissor(I.copy(F).multiplyScalar(G).round())},this.getScissorTest=function(){return ce},this.setScissorTest=function(C){We.setScissorTest(ce=C)},this.setOpaqueSort=function(C){k=C},this.setTransparentSort=function(C){ue=C},this.getClearColor=function(C){return C.copy(Ne.getClearColor())},this.setClearColor=function(){Ne.setClearColor.apply(Ne,arguments)},this.getClearAlpha=function(){return Ne.getClearAlpha()},this.setClearAlpha=function(){Ne.setClearAlpha.apply(Ne,arguments)},this.clear=function(C=!0,q=!0,ie=!0){let ae=0;if(C){let Z=!1;if(D!==null){const Ce=D.texture.format;Z=Ce===Sd||Ce===yd||Ce===xd}if(Z){const Ce=D.texture.type,Ue=Ce===Hi||Ce===Jr||Ce===Qa||Ce===ia||Ce===_d||Ce===vd,He=Ne.getClearColor(),Re=Ne.getClearAlpha(),nt=He.r,et=He.g,$e=He.b;Ue?(M[0]=nt,M[1]=et,M[2]=$e,M[3]=Re,z.clearBufferuiv(z.COLOR,0,M)):(T[0]=nt,T[1]=et,T[2]=$e,T[3]=Re,z.clearBufferiv(z.COLOR,0,T))}else ae|=z.COLOR_BUFFER_BIT}q&&(ae|=z.DEPTH_BUFFER_BIT),ie&&(ae|=z.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),z.clear(ae)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",de,!1),t.removeEventListener("webglcontextrestored",he,!1),t.removeEventListener("webglcontextcreationerror",Te,!1),be.dispose(),De.dispose(),tt.dispose(),U.dispose(),A.dispose(),ye.dispose(),lt.dispose(),Tt.dispose(),me.dispose(),we.dispose(),we.removeEventListener("sessionstart",qt),we.removeEventListener("sessionend",Vi),Qn.stop()};function de(C){C.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),L=!0}function he(){console.log("THREE.WebGLRenderer: Context Restored."),L=!1;const C=Lt.autoReset,q=Ee.enabled,ie=Ee.autoUpdate,ae=Ee.needsUpdate,Z=Ee.type;X(),Lt.autoReset=C,Ee.enabled=q,Ee.autoUpdate=ie,Ee.needsUpdate=ae,Ee.type=Z}function Te(C){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",C.statusMessage)}function qe(C){const q=C.target;q.removeEventListener("dispose",qe),pe(q)}function pe(C){Ke(C),tt.remove(C)}function Ke(C){const q=tt.get(C).programs;q!==void 0&&(q.forEach(function(ie){me.releaseProgram(ie)}),C.isShaderMaterial&&me.releaseShaderCache(C))}this.renderBufferDirect=function(C,q,ie,ae,Z,Ce){q===null&&(q=ke);const Ue=Z.isMesh&&Z.matrixWorld.determinant()<0,He=lu(C,q,ie,ae,Z);We.setMaterial(ae,Ue);let Re=ie.index,nt=1;if(ae.wireframe===!0){if(Re=_e.getWireframeAttribute(ie),Re===void 0)return;nt=2}const et=ie.drawRange,$e=ie.attributes.position;let xt=et.start*nt,kt=(et.start+et.count)*nt;Ce!==null&&(xt=Math.max(xt,Ce.start*nt),kt=Math.min(kt,(Ce.start+Ce.count)*nt)),Re!==null?(xt=Math.max(xt,0),kt=Math.min(kt,Re.count)):$e!=null&&(xt=Math.max(xt,0),kt=Math.min(kt,$e.count));const It=kt-xt;if(It<0||It===1/0)return;lt.setup(Z,ae,He,ie,Re);let tn,ct=Je;if(Re!==null&&(tn=se.get(Re),ct=Oe,ct.setIndex(tn)),Z.isMesh)ae.wireframe===!0?(We.setLineWidth(ae.wireframeLinewidth*wt()),ct.setMode(z.LINES)):ct.setMode(z.TRIANGLES);else if(Z.isLine){let Xe=ae.linewidth;Xe===void 0&&(Xe=1),We.setLineWidth(Xe*wt()),Z.isLineSegments?ct.setMode(z.LINES):Z.isLineLoop?ct.setMode(z.LINE_LOOP):ct.setMode(z.LINE_STRIP)}else Z.isPoints?ct.setMode(z.POINTS):Z.isSprite&&ct.setMode(z.TRIANGLES);if(Z.isBatchedMesh)if(Z._multiDrawInstances!==null)ct.renderMultiDrawInstances(Z._multiDrawStarts,Z._multiDrawCounts,Z._multiDrawCount,Z._multiDrawInstances);else if(vt.get("WEBGL_multi_draw"))ct.renderMultiDraw(Z._multiDrawStarts,Z._multiDrawCounts,Z._multiDrawCount);else{const Xe=Z._multiDrawStarts,Ct=Z._multiDrawCounts,Mt=Z._multiDrawCount,Nn=Re?se.get(Re).bytesPerElement:1,Xi=tt.get(ae).currentProgram.getUniforms();for(let nn=0;nn<Mt;nn++)Xi.setValue(z,"_gl_DrawID",nn),ct.render(Xe[nn]/Nn,Ct[nn])}else if(Z.isInstancedMesh)ct.renderInstances(xt,It,Z.count);else if(ie.isInstancedBufferGeometry){const Xe=ie._maxInstanceCount!==void 0?ie._maxInstanceCount:1/0,Ct=Math.min(ie.instanceCount,Xe);ct.renderInstances(xt,It,Ct)}else ct.render(xt,It)};function mt(C,q,ie){C.transparent===!0&&C.side===ki&&C.forceSinglePass===!1?(C.side=Ln,C.needsUpdate=!0,Wi(C,q,ie),C.side=Mr,C.needsUpdate=!0,Wi(C,q,ie),C.side=ki):Wi(C,q,ie)}this.compile=function(C,q,ie=null){ie===null&&(ie=C),x=De.get(ie),x.init(q),P.push(x),ie.traverseVisible(function(Z){Z.isLight&&Z.layers.test(q.layers)&&(x.pushLight(Z),Z.castShadow&&x.pushShadow(Z))}),C!==ie&&C.traverseVisible(function(Z){Z.isLight&&Z.layers.test(q.layers)&&(x.pushLight(Z),Z.castShadow&&x.pushShadow(Z))}),x.setupLights();const ae=new Set;return C.traverse(function(Z){const Ce=Z.material;if(Ce)if(Array.isArray(Ce))for(let Ue=0;Ue<Ce.length;Ue++){const He=Ce[Ue];mt(He,ie,Z),ae.add(He)}else mt(Ce,ie,Z),ae.add(Ce)}),P.pop(),x=null,ae},this.compileAsync=function(C,q,ie=null){const ae=this.compile(C,q,ie);return new Promise(Z=>{function Ce(){if(ae.forEach(function(Ue){tt.get(Ue).currentProgram.isReady()&&ae.delete(Ue)}),ae.size===0){Z(C);return}setTimeout(Ce,10)}vt.get("KHR_parallel_shader_compile")!==null?Ce():setTimeout(Ce,10)})};let ut=null;function Yt(C){ut&&ut(C)}function qt(){Qn.stop()}function Vi(){Qn.start()}const Qn=new Kg;Qn.setAnimationLoop(Yt),typeof self<"u"&&Qn.setContext(self),this.setAnimationLoop=function(C){ut=C,we.setAnimationLoop(C),C===null?Qn.stop():Qn.start()},we.addEventListener("sessionstart",qt),we.addEventListener("sessionend",Vi),this.render=function(C,q){if(q!==void 0&&q.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(L===!0)return;if(C.matrixWorldAutoUpdate===!0&&C.updateMatrixWorld(),q.parent===null&&q.matrixWorldAutoUpdate===!0&&q.updateMatrixWorld(),we.enabled===!0&&we.isPresenting===!0&&(we.cameraAutoUpdate===!0&&we.updateCamera(q),q=we.getCamera()),C.isScene===!0&&C.onBeforeRender(R,C,q,D),x=De.get(C,P.length),x.init(q),P.push(x),xe.multiplyMatrices(q.projectionMatrix,q.matrixWorldInverse),Ie.setFromProjectionMatrix(xe),fe=this.localClippingEnabled,te=ot.init(this.clippingPlanes,fe),S=be.get(C,_.length),S.init(),_.push(S),we.enabled===!0&&we.isPresenting===!0){const Ce=R.xr.getDepthSensingMesh();Ce!==null&&Si(Ce,q,-1/0,R.sortObjects)}Si(C,q,0,R.sortObjects),S.finish(),R.sortObjects===!0&&S.sort(k,ue),Ye=we.enabled===!1||we.isPresenting===!1||we.hasDepthSensing()===!1,Ye&&Ne.addToRenderList(S,C),this.info.render.frame++,te===!0&&ot.beginShadows();const ie=x.state.shadowsArray;Ee.render(ie,C,q),te===!0&&ot.endShadows(),this.info.autoReset===!0&&this.info.reset();const ae=S.opaque,Z=S.transmissive;if(x.setupLights(),q.isArrayCamera){const Ce=q.cameras;if(Z.length>0)for(let Ue=0,He=Ce.length;Ue<He;Ue++){const Re=Ce[Ue];Gi(ae,Z,C,Re)}Ye&&Ne.render(C);for(let Ue=0,He=Ce.length;Ue<He;Ue++){const Re=Ce[Ue];io(S,C,Re,Re.viewport)}}else Z.length>0&&Gi(ae,Z,C,q),Ye&&Ne.render(C),io(S,C,q);D!==null&&(rt.updateMultisampleRenderTarget(D),rt.updateRenderTargetMipmap(D)),C.isScene===!0&&C.onAfterRender(R,C,q),lt.resetDefaultState(),j=-1,b=null,P.pop(),P.length>0?(x=P[P.length-1],te===!0&&ot.setGlobalState(R.clippingPlanes,x.state.camera)):x=null,_.pop(),_.length>0?S=_[_.length-1]:S=null};function Si(C,q,ie,ae){if(C.visible===!1)return;if(C.layers.test(q.layers)){if(C.isGroup)ie=C.renderOrder;else if(C.isLOD)C.autoUpdate===!0&&C.update(q);else if(C.isLight)x.pushLight(C),C.castShadow&&x.pushShadow(C);else if(C.isSprite){if(!C.frustumCulled||Ie.intersectsSprite(C)){ae&&Le.setFromMatrixPosition(C.matrixWorld).applyMatrix4(xe);const Ue=ye.update(C),He=C.material;He.visible&&S.push(C,Ue,He,ie,Le.z,null)}}else if((C.isMesh||C.isLine||C.isPoints)&&(!C.frustumCulled||Ie.intersectsObject(C))){const Ue=ye.update(C),He=C.material;if(ae&&(C.boundingSphere!==void 0?(C.boundingSphere===null&&C.computeBoundingSphere(),Le.copy(C.boundingSphere.center)):(Ue.boundingSphere===null&&Ue.computeBoundingSphere(),Le.copy(Ue.boundingSphere.center)),Le.applyMatrix4(C.matrixWorld).applyMatrix4(xe)),Array.isArray(He)){const Re=Ue.groups;for(let nt=0,et=Re.length;nt<et;nt++){const $e=Re[nt],xt=He[$e.materialIndex];xt&&xt.visible&&S.push(C,Ue,xt,ie,Le.z,$e)}}else He.visible&&S.push(C,Ue,He,ie,Le.z,null)}}const Ce=C.children;for(let Ue=0,He=Ce.length;Ue<He;Ue++)Si(Ce[Ue],q,ie,ae)}function io(C,q,ie,ae){const Z=C.opaque,Ce=C.transmissive,Ue=C.transparent;x.setupLightsView(ie),te===!0&&ot.setGlobalState(R.clippingPlanes,ie),ae&&We.viewport(w.copy(ae)),Z.length>0&&Mi(Z,q,ie),Ce.length>0&&Mi(Ce,q,ie),Ue.length>0&&Mi(Ue,q,ie),We.buffers.depth.setTest(!0),We.buffers.depth.setMask(!0),We.buffers.color.setMask(!0),We.setPolygonOffset(!1)}function Gi(C,q,ie,ae){if((ie.isScene===!0?ie.overrideMaterial:null)!==null)return;x.state.transmissionRenderTarget[ae.id]===void 0&&(x.state.transmissionRenderTarget[ae.id]=new es(1,1,{generateMipmaps:!0,type:vt.has("EXT_color_buffer_half_float")||vt.has("EXT_color_buffer_float")?eo:Hi,minFilter:Kr,samples:4,stencilBuffer:u,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:At.workingColorSpace}));const Ce=x.state.transmissionRenderTarget[ae.id],Ue=ae.viewport||w;Ce.setSize(Ue.z,Ue.w);const He=R.getRenderTarget();R.setRenderTarget(Ce),R.getClearColor(K),oe=R.getClearAlpha(),oe<1&&R.setClearColor(16777215,.5),R.clear(),Ye&&Ne.render(ie);const Re=R.toneMapping;R.toneMapping=yr;const nt=ae.viewport;if(ae.viewport!==void 0&&(ae.viewport=void 0),x.setupLightsView(ae),te===!0&&ot.setGlobalState(R.clippingPlanes,ae),Mi(C,ie,ae),rt.updateMultisampleRenderTarget(Ce),rt.updateRenderTargetMipmap(Ce),vt.has("WEBGL_multisampled_render_to_texture")===!1){let et=!1;for(let $e=0,xt=q.length;$e<xt;$e++){const kt=q[$e],It=kt.object,tn=kt.geometry,ct=kt.material,Xe=kt.group;if(ct.side===ki&&It.layers.test(ae.layers)){const Ct=ct.side;ct.side=Ln,ct.needsUpdate=!0,Tr(It,ie,ae,tn,ct,Xe),ct.side=Ct,ct.needsUpdate=!0,et=!0}}et===!0&&(rt.updateMultisampleRenderTarget(Ce),rt.updateRenderTargetMipmap(Ce))}R.setRenderTarget(He),R.setClearColor(K,oe),nt!==void 0&&(ae.viewport=nt),R.toneMapping=Re}function Mi(C,q,ie){const ae=q.isScene===!0?q.overrideMaterial:null;for(let Z=0,Ce=C.length;Z<Ce;Z++){const Ue=C[Z],He=Ue.object,Re=Ue.geometry,nt=ae===null?Ue.material:ae,et=Ue.group;He.layers.test(ie.layers)&&Tr(He,q,ie,Re,nt,et)}}function Tr(C,q,ie,ae,Z,Ce){C.onBeforeRender(R,q,ie,ae,Z,Ce),C.modelViewMatrix.multiplyMatrices(ie.matrixWorldInverse,C.matrixWorld),C.normalMatrix.getNormalMatrix(C.modelViewMatrix),Z.transparent===!0&&Z.side===ki&&Z.forceSinglePass===!1?(Z.side=Ln,Z.needsUpdate=!0,R.renderBufferDirect(ie,q,ae,Z,C,Ce),Z.side=Mr,Z.needsUpdate=!0,R.renderBufferDirect(ie,q,ae,Z,C,Ce),Z.side=ki):R.renderBufferDirect(ie,q,ae,Z,C,Ce),C.onAfterRender(R,q,ie,ae,Z,Ce)}function Wi(C,q,ie){q.isScene!==!0&&(q=ke);const ae=tt.get(C),Z=x.state.lights,Ce=x.state.shadowsArray,Ue=Z.state.version,He=me.getParameters(C,Z.state,Ce,q,ie),Re=me.getProgramCacheKey(He);let nt=ae.programs;ae.environment=C.isMeshStandardMaterial?q.environment:null,ae.fog=q.fog,ae.envMap=(C.isMeshStandardMaterial?A:U).get(C.envMap||ae.environment),ae.envMapRotation=ae.environment!==null&&C.envMap===null?q.environmentRotation:C.envMapRotation,nt===void 0&&(C.addEventListener("dispose",qe),nt=new Map,ae.programs=nt);let et=nt.get(Re);if(et!==void 0){if(ae.currentProgram===et&&ae.lightsStateVersion===Ue)return so(C,He),et}else He.uniforms=me.getUniforms(C),C.onBeforeCompile(He,R),et=me.acquireProgram(He,Re),nt.set(Re,et),ae.uniforms=He.uniforms;const $e=ae.uniforms;return(!C.isShaderMaterial&&!C.isRawShaderMaterial||C.clipping===!0)&&($e.clippingPlanes=ot.uniform),so(C,He),ae.needsLights=ao(C),ae.lightsStateVersion=Ue,ae.needsLights&&($e.ambientLightColor.value=Z.state.ambient,$e.lightProbe.value=Z.state.probe,$e.directionalLights.value=Z.state.directional,$e.directionalLightShadows.value=Z.state.directionalShadow,$e.spotLights.value=Z.state.spot,$e.spotLightShadows.value=Z.state.spotShadow,$e.rectAreaLights.value=Z.state.rectArea,$e.ltc_1.value=Z.state.rectAreaLTC1,$e.ltc_2.value=Z.state.rectAreaLTC2,$e.pointLights.value=Z.state.point,$e.pointLightShadows.value=Z.state.pointShadow,$e.hemisphereLights.value=Z.state.hemi,$e.directionalShadowMap.value=Z.state.directionalShadowMap,$e.directionalShadowMatrix.value=Z.state.directionalShadowMatrix,$e.spotShadowMap.value=Z.state.spotShadowMap,$e.spotLightMatrix.value=Z.state.spotLightMatrix,$e.spotLightMap.value=Z.state.spotLightMap,$e.pointShadowMap.value=Z.state.pointShadowMap,$e.pointShadowMatrix.value=Z.state.pointShadowMatrix),ae.currentProgram=et,ae.uniformsList=null,et}function ro(C){if(C.uniformsList===null){const q=C.currentProgram.getUniforms();C.uniformsList=Xl.seqWithValue(q.seq,C.uniforms)}return C.uniformsList}function so(C,q){const ie=tt.get(C);ie.outputColorSpace=q.outputColorSpace,ie.batching=q.batching,ie.batchingColor=q.batchingColor,ie.instancing=q.instancing,ie.instancingColor=q.instancingColor,ie.instancingMorph=q.instancingMorph,ie.skinning=q.skinning,ie.morphTargets=q.morphTargets,ie.morphNormals=q.morphNormals,ie.morphColors=q.morphColors,ie.morphTargetsCount=q.morphTargetsCount,ie.numClippingPlanes=q.numClippingPlanes,ie.numIntersection=q.numClipIntersection,ie.vertexAlphas=q.vertexAlphas,ie.vertexTangents=q.vertexTangents,ie.toneMapping=q.toneMapping}function lu(C,q,ie,ae,Z){q.isScene!==!0&&(q=ke),rt.resetTextureUnits();const Ce=q.fog,Ue=ae.isMeshStandardMaterial?q.environment:null,He=D===null?R.outputColorSpace:D.isXRRenderTarget===!0?D.texture.colorSpace:wr,Re=(ae.isMeshStandardMaterial?A:U).get(ae.envMap||Ue),nt=ae.vertexColors===!0&&!!ie.attributes.color&&ie.attributes.color.itemSize===4,et=!!ie.attributes.tangent&&(!!ae.normalMap||ae.anisotropy>0),$e=!!ie.morphAttributes.position,xt=!!ie.morphAttributes.normal,kt=!!ie.morphAttributes.color;let It=yr;ae.toneMapped&&(D===null||D.isXRRenderTarget===!0)&&(It=R.toneMapping);const tn=ie.morphAttributes.position||ie.morphAttributes.normal||ie.morphAttributes.color,ct=tn!==void 0?tn.length:0,Xe=tt.get(ae),Ct=x.state.lights;if(te===!0&&(fe===!0||C!==b)){const wn=C===b&&ae.id===j;ot.setState(ae,C,wn)}let Mt=!1;ae.version===Xe.__version?(Xe.needsLights&&Xe.lightsStateVersion!==Ct.state.version||Xe.outputColorSpace!==He||Z.isBatchedMesh&&Xe.batching===!1||!Z.isBatchedMesh&&Xe.batching===!0||Z.isBatchedMesh&&Xe.batchingColor===!0&&Z.colorTexture===null||Z.isBatchedMesh&&Xe.batchingColor===!1&&Z.colorTexture!==null||Z.isInstancedMesh&&Xe.instancing===!1||!Z.isInstancedMesh&&Xe.instancing===!0||Z.isSkinnedMesh&&Xe.skinning===!1||!Z.isSkinnedMesh&&Xe.skinning===!0||Z.isInstancedMesh&&Xe.instancingColor===!0&&Z.instanceColor===null||Z.isInstancedMesh&&Xe.instancingColor===!1&&Z.instanceColor!==null||Z.isInstancedMesh&&Xe.instancingMorph===!0&&Z.morphTexture===null||Z.isInstancedMesh&&Xe.instancingMorph===!1&&Z.morphTexture!==null||Xe.envMap!==Re||ae.fog===!0&&Xe.fog!==Ce||Xe.numClippingPlanes!==void 0&&(Xe.numClippingPlanes!==ot.numPlanes||Xe.numIntersection!==ot.numIntersection)||Xe.vertexAlphas!==nt||Xe.vertexTangents!==et||Xe.morphTargets!==$e||Xe.morphNormals!==xt||Xe.morphColors!==kt||Xe.toneMapping!==It||Xe.morphTargetsCount!==ct)&&(Mt=!0):(Mt=!0,Xe.__version=ae.version);let Nn=Xe.currentProgram;Mt===!0&&(Nn=Wi(ae,q,Z));let Xi=!1,nn=!1,ji=!1;const Nt=Nn.getUniforms(),Dn=Xe.uniforms;if(We.useProgram(Nn.program)&&(Xi=!0,nn=!0,ji=!0),ae.id!==j&&(j=ae.id,nn=!0),Xi||b!==C){Nt.setValue(z,"projectionMatrix",C.projectionMatrix),Nt.setValue(z,"viewMatrix",C.matrixWorldInverse);const wn=Nt.map.cameraPosition;wn!==void 0&&wn.setValue(z,Me.setFromMatrixPosition(C.matrixWorld)),yt.logarithmicDepthBuffer&&Nt.setValue(z,"logDepthBufFC",2/(Math.log(C.far+1)/Math.LN2)),(ae.isMeshPhongMaterial||ae.isMeshToonMaterial||ae.isMeshLambertMaterial||ae.isMeshBasicMaterial||ae.isMeshStandardMaterial||ae.isShaderMaterial)&&Nt.setValue(z,"isOrthographic",C.isOrthographicCamera===!0),b!==C&&(b=C,nn=!0,ji=!0)}if(Z.isSkinnedMesh){Nt.setOptional(z,Z,"bindMatrix"),Nt.setOptional(z,Z,"bindMatrixInverse");const wn=Z.skeleton;wn&&(wn.boneTexture===null&&wn.computeBoneTexture(),Nt.setValue(z,"boneTexture",wn.boneTexture,rt))}Z.isBatchedMesh&&(Nt.setOptional(z,Z,"batchingTexture"),Nt.setValue(z,"batchingTexture",Z._matricesTexture,rt),Nt.setOptional(z,Z,"batchingIdTexture"),Nt.setValue(z,"batchingIdTexture",Z._indirectTexture,rt),Nt.setOptional(z,Z,"batchingColorTexture"),Z._colorsTexture!==null&&Nt.setValue(z,"batchingColorTexture",Z._colorsTexture,rt));const la=ie.morphAttributes;if((la.position!==void 0||la.normal!==void 0||la.color!==void 0)&&pt.update(Z,ie,Nn),(nn||Xe.receiveShadow!==Z.receiveShadow)&&(Xe.receiveShadow=Z.receiveShadow,Nt.setValue(z,"receiveShadow",Z.receiveShadow)),ae.isMeshGouraudMaterial&&ae.envMap!==null&&(Dn.envMap.value=Re,Dn.flipEnvMap.value=Re.isCubeTexture&&Re.isRenderTargetTexture===!1?-1:1),ae.isMeshStandardMaterial&&ae.envMap===null&&q.environment!==null&&(Dn.envMapIntensity.value=q.environmentIntensity),nn&&(Nt.setValue(z,"toneMappingExposure",R.toneMappingExposure),Xe.needsLights&&Ei(Dn,ji),Ce&&ae.fog===!0&&je.refreshFogUniforms(Dn,Ce),je.refreshMaterialUniforms(Dn,ae,G,B,x.state.transmissionRenderTarget[C.id]),Xl.upload(z,ro(Xe),Dn,rt)),ae.isShaderMaterial&&ae.uniformsNeedUpdate===!0&&(Xl.upload(z,ro(Xe),Dn,rt),ae.uniformsNeedUpdate=!1),ae.isSpriteMaterial&&Nt.setValue(z,"center",Z.center),Nt.setValue(z,"modelViewMatrix",Z.modelViewMatrix),Nt.setValue(z,"normalMatrix",Z.normalMatrix),Nt.setValue(z,"modelMatrix",Z.matrixWorld),ae.isShaderMaterial||ae.isRawShaderMaterial){const wn=ae.uniformsGroups;for(let Ar=0,oo=wn.length;Ar<oo;Ar++){const is=wn[Ar];Tt.update(is,Nn),Tt.bind(is,Nn)}}return Nn}function Ei(C,q){C.ambientLightColor.needsUpdate=q,C.lightProbe.needsUpdate=q,C.directionalLights.needsUpdate=q,C.directionalLightShadows.needsUpdate=q,C.pointLights.needsUpdate=q,C.pointLightShadows.needsUpdate=q,C.spotLights.needsUpdate=q,C.spotLightShadows.needsUpdate=q,C.rectAreaLights.needsUpdate=q,C.hemisphereLights.needsUpdate=q}function ao(C){return C.isMeshLambertMaterial||C.isMeshToonMaterial||C.isMeshPhongMaterial||C.isMeshStandardMaterial||C.isShadowMaterial||C.isShaderMaterial&&C.lights===!0}this.getActiveCubeFace=function(){return $},this.getActiveMipmapLevel=function(){return O},this.getRenderTarget=function(){return D},this.setRenderTargetTextures=function(C,q,ie){tt.get(C.texture).__webglTexture=q,tt.get(C.depthTexture).__webglTexture=ie;const ae=tt.get(C);ae.__hasExternalTextures=!0,ae.__autoAllocateDepthBuffer=ie===void 0,ae.__autoAllocateDepthBuffer||vt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),ae.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(C,q){const ie=tt.get(C);ie.__webglFramebuffer=q,ie.__useDefaultFramebuffer=q===void 0},this.setRenderTarget=function(C,q=0,ie=0){D=C,$=q,O=ie;let ae=!0,Z=null,Ce=!1,Ue=!1;if(C){const Re=tt.get(C);Re.__useDefaultFramebuffer!==void 0?(We.bindFramebuffer(z.FRAMEBUFFER,null),ae=!1):Re.__webglFramebuffer===void 0?rt.setupRenderTarget(C):Re.__hasExternalTextures&&rt.rebindTextures(C,tt.get(C.texture).__webglTexture,tt.get(C.depthTexture).__webglTexture);const nt=C.texture;(nt.isData3DTexture||nt.isDataArrayTexture||nt.isCompressedArrayTexture)&&(Ue=!0);const et=tt.get(C).__webglFramebuffer;C.isWebGLCubeRenderTarget?(Array.isArray(et[q])?Z=et[q][ie]:Z=et[q],Ce=!0):C.samples>0&&rt.useMultisampledRTT(C)===!1?Z=tt.get(C).__webglMultisampledFramebuffer:Array.isArray(et)?Z=et[ie]:Z=et,w.copy(C.viewport),I.copy(C.scissor),Y=C.scissorTest}else w.copy(le).multiplyScalar(G).floor(),I.copy(F).multiplyScalar(G).floor(),Y=ce;if(We.bindFramebuffer(z.FRAMEBUFFER,Z)&&ae&&We.drawBuffers(C,Z),We.viewport(w),We.scissor(I),We.setScissorTest(Y),Ce){const Re=tt.get(C.texture);z.framebufferTexture2D(z.FRAMEBUFFER,z.COLOR_ATTACHMENT0,z.TEXTURE_CUBE_MAP_POSITIVE_X+q,Re.__webglTexture,ie)}else if(Ue){const Re=tt.get(C.texture),nt=q||0;z.framebufferTextureLayer(z.FRAMEBUFFER,z.COLOR_ATTACHMENT0,Re.__webglTexture,ie||0,nt)}j=-1},this.readRenderTargetPixels=function(C,q,ie,ae,Z,Ce,Ue){if(!(C&&C.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let He=tt.get(C).__webglFramebuffer;if(C.isWebGLCubeRenderTarget&&Ue!==void 0&&(He=He[Ue]),He){We.bindFramebuffer(z.FRAMEBUFFER,He);try{const Re=C.texture,nt=Re.format,et=Re.type;if(!yt.textureFormatReadable(nt)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!yt.textureTypeReadable(et)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}q>=0&&q<=C.width-ae&&ie>=0&&ie<=C.height-Z&&z.readPixels(q,ie,ae,Z,st.convert(nt),st.convert(et),Ce)}finally{const Re=D!==null?tt.get(D).__webglFramebuffer:null;We.bindFramebuffer(z.FRAMEBUFFER,Re)}}},this.readRenderTargetPixelsAsync=async function(C,q,ie,ae,Z,Ce,Ue){if(!(C&&C.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let He=tt.get(C).__webglFramebuffer;if(C.isWebGLCubeRenderTarget&&Ue!==void 0&&(He=He[Ue]),He){We.bindFramebuffer(z.FRAMEBUFFER,He);try{const Re=C.texture,nt=Re.format,et=Re.type;if(!yt.textureFormatReadable(nt))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!yt.textureTypeReadable(et))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(q>=0&&q<=C.width-ae&&ie>=0&&ie<=C.height-Z){const $e=z.createBuffer();z.bindBuffer(z.PIXEL_PACK_BUFFER,$e),z.bufferData(z.PIXEL_PACK_BUFFER,Ce.byteLength,z.STREAM_READ),z.readPixels(q,ie,ae,Z,st.convert(nt),st.convert(et),0),z.flush();const xt=z.fenceSync(z.SYNC_GPU_COMMANDS_COMPLETE,0);await Z0(z,xt,4);try{z.bindBuffer(z.PIXEL_PACK_BUFFER,$e),z.getBufferSubData(z.PIXEL_PACK_BUFFER,0,Ce)}finally{z.deleteBuffer($e),z.deleteSync(xt)}return Ce}}finally{const Re=D!==null?tt.get(D).__webglFramebuffer:null;We.bindFramebuffer(z.FRAMEBUFFER,Re)}}},this.copyFramebufferToTexture=function(C,q=null,ie=0){C.isTexture!==!0&&(Js("WebGLRenderer: copyFramebufferToTexture function signature has changed."),q=arguments[0]||null,C=arguments[1]);const ae=Math.pow(2,-ie),Z=Math.floor(C.image.width*ae),Ce=Math.floor(C.image.height*ae),Ue=q!==null?q.x:0,He=q!==null?q.y:0;rt.setTexture2D(C,0),z.copyTexSubImage2D(z.TEXTURE_2D,ie,0,0,Ue,He,Z,Ce),We.unbindTexture()},this.copyTextureToTexture=function(C,q,ie=null,ae=null,Z=0){C.isTexture!==!0&&(Js("WebGLRenderer: copyTextureToTexture function signature has changed."),ae=arguments[0]||null,C=arguments[1],q=arguments[2],Z=arguments[3]||0,ie=null);let Ce,Ue,He,Re,nt,et;ie!==null?(Ce=ie.max.x-ie.min.x,Ue=ie.max.y-ie.min.y,He=ie.min.x,Re=ie.min.y):(Ce=C.image.width,Ue=C.image.height,He=0,Re=0),ae!==null?(nt=ae.x,et=ae.y):(nt=0,et=0);const $e=st.convert(q.format),xt=st.convert(q.type);rt.setTexture2D(q,0),z.pixelStorei(z.UNPACK_FLIP_Y_WEBGL,q.flipY),z.pixelStorei(z.UNPACK_PREMULTIPLY_ALPHA_WEBGL,q.premultiplyAlpha),z.pixelStorei(z.UNPACK_ALIGNMENT,q.unpackAlignment);const kt=z.getParameter(z.UNPACK_ROW_LENGTH),It=z.getParameter(z.UNPACK_IMAGE_HEIGHT),tn=z.getParameter(z.UNPACK_SKIP_PIXELS),ct=z.getParameter(z.UNPACK_SKIP_ROWS),Xe=z.getParameter(z.UNPACK_SKIP_IMAGES),Ct=C.isCompressedTexture?C.mipmaps[Z]:C.image;z.pixelStorei(z.UNPACK_ROW_LENGTH,Ct.width),z.pixelStorei(z.UNPACK_IMAGE_HEIGHT,Ct.height),z.pixelStorei(z.UNPACK_SKIP_PIXELS,He),z.pixelStorei(z.UNPACK_SKIP_ROWS,Re),C.isDataTexture?z.texSubImage2D(z.TEXTURE_2D,Z,nt,et,Ce,Ue,$e,xt,Ct.data):C.isCompressedTexture?z.compressedTexSubImage2D(z.TEXTURE_2D,Z,nt,et,Ct.width,Ct.height,$e,Ct.data):z.texSubImage2D(z.TEXTURE_2D,Z,nt,et,Ce,Ue,$e,xt,Ct),z.pixelStorei(z.UNPACK_ROW_LENGTH,kt),z.pixelStorei(z.UNPACK_IMAGE_HEIGHT,It),z.pixelStorei(z.UNPACK_SKIP_PIXELS,tn),z.pixelStorei(z.UNPACK_SKIP_ROWS,ct),z.pixelStorei(z.UNPACK_SKIP_IMAGES,Xe),Z===0&&q.generateMipmaps&&z.generateMipmap(z.TEXTURE_2D),We.unbindTexture()},this.copyTextureToTexture3D=function(C,q,ie=null,ae=null,Z=0){C.isTexture!==!0&&(Js("WebGLRenderer: copyTextureToTexture3D function signature has changed."),ie=arguments[0]||null,ae=arguments[1]||null,C=arguments[2],q=arguments[3],Z=arguments[4]||0);let Ce,Ue,He,Re,nt,et,$e,xt,kt;const It=C.isCompressedTexture?C.mipmaps[Z]:C.image;ie!==null?(Ce=ie.max.x-ie.min.x,Ue=ie.max.y-ie.min.y,He=ie.max.z-ie.min.z,Re=ie.min.x,nt=ie.min.y,et=ie.min.z):(Ce=It.width,Ue=It.height,He=It.depth,Re=0,nt=0,et=0),ae!==null?($e=ae.x,xt=ae.y,kt=ae.z):($e=0,xt=0,kt=0);const tn=st.convert(q.format),ct=st.convert(q.type);let Xe;if(q.isData3DTexture)rt.setTexture3D(q,0),Xe=z.TEXTURE_3D;else if(q.isDataArrayTexture||q.isCompressedArrayTexture)rt.setTexture2DArray(q,0),Xe=z.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}z.pixelStorei(z.UNPACK_FLIP_Y_WEBGL,q.flipY),z.pixelStorei(z.UNPACK_PREMULTIPLY_ALPHA_WEBGL,q.premultiplyAlpha),z.pixelStorei(z.UNPACK_ALIGNMENT,q.unpackAlignment);const Ct=z.getParameter(z.UNPACK_ROW_LENGTH),Mt=z.getParameter(z.UNPACK_IMAGE_HEIGHT),Nn=z.getParameter(z.UNPACK_SKIP_PIXELS),Xi=z.getParameter(z.UNPACK_SKIP_ROWS),nn=z.getParameter(z.UNPACK_SKIP_IMAGES);z.pixelStorei(z.UNPACK_ROW_LENGTH,It.width),z.pixelStorei(z.UNPACK_IMAGE_HEIGHT,It.height),z.pixelStorei(z.UNPACK_SKIP_PIXELS,Re),z.pixelStorei(z.UNPACK_SKIP_ROWS,nt),z.pixelStorei(z.UNPACK_SKIP_IMAGES,et),C.isDataTexture||C.isData3DTexture?z.texSubImage3D(Xe,Z,$e,xt,kt,Ce,Ue,He,tn,ct,It.data):q.isCompressedArrayTexture?z.compressedTexSubImage3D(Xe,Z,$e,xt,kt,Ce,Ue,He,tn,It.data):z.texSubImage3D(Xe,Z,$e,xt,kt,Ce,Ue,He,tn,ct,It),z.pixelStorei(z.UNPACK_ROW_LENGTH,Ct),z.pixelStorei(z.UNPACK_IMAGE_HEIGHT,Mt),z.pixelStorei(z.UNPACK_SKIP_PIXELS,Nn),z.pixelStorei(z.UNPACK_SKIP_ROWS,Xi),z.pixelStorei(z.UNPACK_SKIP_IMAGES,nn),Z===0&&q.generateMipmaps&&z.generateMipmap(Xe),We.unbindTexture()},this.initRenderTarget=function(C){tt.get(C).__webglFramebuffer===void 0&&rt.setupRenderTarget(C)},this.initTexture=function(C){C.isCubeTexture?rt.setTextureCube(C,0):C.isData3DTexture?rt.setTexture3D(C,0):C.isDataArrayTexture||C.isCompressedArrayTexture?rt.setTexture2DArray(C,0):rt.setTexture2D(C,0),We.unbindTexture()},this.resetState=function(){$=0,O=0,D=null,We.reset(),lt.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return zi}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;const t=this.getContext();t.drawingBufferColorSpace=e===Md?"display-p3":"srgb",t.unpackColorSpace=At.workingColorSpace===nu?"display-p3":"srgb"}}class n_ extends Qt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new yi,this.environmentIntensity=1,this.environmentRotation=new yi,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){const t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}}class ow{constructor(e,t){this.isInterleavedBuffer=!0,this.array=e,this.stride=t,this.count=e!==void 0?e.length/t:0,this.usage=ud,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.version=0,this.uuid=Sr()}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return Js("THREE.InterleavedBuffer: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.array=new e.array.constructor(e.array),this.count=e.count,this.stride=e.stride,this.usage=e.usage,this}copyAt(e,t,r){e*=this.stride,r*=t.stride;for(let o=0,u=this.stride;o<u;o++)this.array[e+o]=t.array[r+o];return this}set(e,t=0){return this.array.set(e,t),this}clone(e){e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Sr()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=this.array.slice(0).buffer);const t=new this.array.constructor(e.arrayBuffers[this.array.buffer._uuid]),r=new this.constructor(t,this.stride);return r.setUsage(this.usage),r}onUpload(e){return this.onUploadCallback=e,this}toJSON(e){return e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Sr()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=Array.from(new Uint32Array(this.array.buffer))),{uuid:this.uuid,buffer:this.array.buffer._uuid,type:this.array.constructor.name,stride:this.stride}}}const yn=new J;class Ql{constructor(e,t,r,o=!1){this.isInterleavedBufferAttribute=!0,this.name="",this.data=e,this.itemSize=t,this.offset=r,this.normalized=o}get count(){return this.data.count}get array(){return this.data.array}set needsUpdate(e){this.data.needsUpdate=e}applyMatrix4(e){for(let t=0,r=this.data.count;t<r;t++)yn.fromBufferAttribute(this,t),yn.applyMatrix4(e),this.setXYZ(t,yn.x,yn.y,yn.z);return this}applyNormalMatrix(e){for(let t=0,r=this.count;t<r;t++)yn.fromBufferAttribute(this,t),yn.applyNormalMatrix(e),this.setXYZ(t,yn.x,yn.y,yn.z);return this}transformDirection(e){for(let t=0,r=this.count;t<r;t++)yn.fromBufferAttribute(this,t),yn.transformDirection(e),this.setXYZ(t,yn.x,yn.y,yn.z);return this}getComponent(e,t){let r=this.array[e*this.data.stride+this.offset+t];return this.normalized&&(r=xi(r,this.array)),r}setComponent(e,t,r){return this.normalized&&(r=Pt(r,this.array)),this.data.array[e*this.data.stride+this.offset+t]=r,this}setX(e,t){return this.normalized&&(t=Pt(t,this.array)),this.data.array[e*this.data.stride+this.offset]=t,this}setY(e,t){return this.normalized&&(t=Pt(t,this.array)),this.data.array[e*this.data.stride+this.offset+1]=t,this}setZ(e,t){return this.normalized&&(t=Pt(t,this.array)),this.data.array[e*this.data.stride+this.offset+2]=t,this}setW(e,t){return this.normalized&&(t=Pt(t,this.array)),this.data.array[e*this.data.stride+this.offset+3]=t,this}getX(e){let t=this.data.array[e*this.data.stride+this.offset];return this.normalized&&(t=xi(t,this.array)),t}getY(e){let t=this.data.array[e*this.data.stride+this.offset+1];return this.normalized&&(t=xi(t,this.array)),t}getZ(e){let t=this.data.array[e*this.data.stride+this.offset+2];return this.normalized&&(t=xi(t,this.array)),t}getW(e){let t=this.data.array[e*this.data.stride+this.offset+3];return this.normalized&&(t=xi(t,this.array)),t}setXY(e,t,r){return e=e*this.data.stride+this.offset,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this}setXYZ(e,t,r,o){return e=e*this.data.stride+this.offset,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array),o=Pt(o,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this.data.array[e+2]=o,this}setXYZW(e,t,r,o,u){return e=e*this.data.stride+this.offset,this.normalized&&(t=Pt(t,this.array),r=Pt(r,this.array),o=Pt(o,this.array),u=Pt(u,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=r,this.data.array[e+2]=o,this.data.array[e+3]=u,this}clone(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.clone(): Cloning an interleaved buffer attribute will de-interleave buffer data.");const t=[];for(let r=0;r<this.count;r++){const o=r*this.data.stride+this.offset;for(let u=0;u<this.itemSize;u++)t.push(this.data.array[o+u])}return new di(new this.array.constructor(t),this.itemSize,this.normalized)}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.clone(e)),new Ql(e.interleavedBuffers[this.data.uuid],this.itemSize,this.offset,this.normalized)}toJSON(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.toJSON(): Serializing an interleaved buffer attribute will de-interleave buffer data.");const t=[];for(let r=0;r<this.count;r++){const o=r*this.data.stride+this.offset;for(let u=0;u<this.itemSize;u++)t.push(this.data.array[o+u])}return{itemSize:this.itemSize,type:this.array.constructor.name,array:t,normalized:this.normalized}}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.toJSON(e)),{isInterleavedBufferAttribute:!0,itemSize:this.itemSize,data:this.data.uuid,offset:this.offset,normalized:this.normalized}}}class i_ extends ns{constructor(e){super(),this.isSpriteMaterial=!0,this.type="SpriteMaterial",this.color=new _t(16777215),this.map=null,this.alphaMap=null,this.rotation=0,this.sizeAttenuation=!0,this.transparent=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.rotation=e.rotation,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}}let Gs;const Ya=new J,Ws=new J,Xs=new J,js=new ft,qa=new ft,r_=new Vt,Nl=new J,$a=new J,Dl=new J,ag=new ft,yf=new ft,og=new ft;class lw extends Qt{constructor(e=new i_){if(super(),this.isSprite=!0,this.type="Sprite",Gs===void 0){Gs=new Hn;const t=new Float32Array([-.5,-.5,0,0,0,.5,-.5,0,1,0,.5,.5,0,1,1,-.5,.5,0,0,1]),r=new ow(t,5);Gs.setIndex([0,1,2,0,2,3]),Gs.setAttribute("position",new Ql(r,3,0,!1)),Gs.setAttribute("uv",new Ql(r,2,3,!1))}this.geometry=Gs,this.material=e,this.center=new ft(.5,.5)}raycast(e,t){e.camera===null&&console.error('THREE.Sprite: "Raycaster.camera" needs to be set in order to raycast against sprites.'),Ws.setFromMatrixScale(this.matrixWorld),r_.copy(e.camera.matrixWorld),this.modelViewMatrix.multiplyMatrices(e.camera.matrixWorldInverse,this.matrixWorld),Xs.setFromMatrixPosition(this.modelViewMatrix),e.camera.isPerspectiveCamera&&this.material.sizeAttenuation===!1&&Ws.multiplyScalar(-Xs.z);const r=this.material.rotation;let o,u;r!==0&&(u=Math.cos(r),o=Math.sin(r));const c=this.center;Il(Nl.set(-.5,-.5,0),Xs,c,Ws,o,u),Il($a.set(.5,-.5,0),Xs,c,Ws,o,u),Il(Dl.set(.5,.5,0),Xs,c,Ws,o,u),ag.set(0,0),yf.set(1,0),og.set(1,1);let d=e.ray.intersectTriangle(Nl,$a,Dl,!1,Ya);if(d===null&&(Il($a.set(-.5,.5,0),Xs,c,Ws,o,u),yf.set(0,1),d=e.ray.intersectTriangle(Nl,Dl,$a,!1,Ya),d===null))return;const h=e.ray.origin.distanceTo(Ya);h<e.near||h>e.far||t.push({distance:h,point:Ya.clone(),uv:ci.getInterpolation(Ya,Nl,$a,Dl,ag,yf,og,new ft),face:null,object:this})}copy(e,t){return super.copy(e,t),e.center!==void 0&&this.center.copy(e.center),this.material=e.material,this}}function Il(s,e,t,r,o,u){js.subVectors(s,t).addScalar(.5).multiply(r),o!==void 0?(qa.x=u*js.x-o*js.y,qa.y=o*js.x+u*js.y):qa.copy(js),s.copy(e),s.x+=qa.x,s.y+=qa.y,s.applyMatrix4(r_)}class au extends ns{constructor(e){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new _t(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.linewidth=e.linewidth,this.linecap=e.linecap,this.linejoin=e.linejoin,this.fog=e.fog,this}}const Jl=new J,eu=new J,lg=new Vt,Ka=new Vg,Ul=new iu,Sf=new J,ug=new J;class s_ extends Qt{constructor(e=new Hn,t=new au){super(),this.isLine=!0,this.type="Line",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,r=[0];for(let o=1,u=t.count;o<u;o++)Jl.fromBufferAttribute(t,o-1),eu.fromBufferAttribute(t,o),r[o]=r[o-1],r[o]+=Jl.distanceTo(eu);e.setAttribute("lineDistance",new _n(r,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(e,t){const r=this.geometry,o=this.matrixWorld,u=e.params.Line.threshold,c=r.drawRange;if(r.boundingSphere===null&&r.computeBoundingSphere(),Ul.copy(r.boundingSphere),Ul.applyMatrix4(o),Ul.radius+=u,e.ray.intersectsSphere(Ul)===!1)return;lg.copy(o).invert(),Ka.copy(e.ray).applyMatrix4(lg);const d=u/((this.scale.x+this.scale.y+this.scale.z)/3),h=d*d,m=this.isLineSegments?2:1,g=r.index,v=r.attributes.position;if(g!==null){const M=Math.max(0,c.start),T=Math.min(g.count,c.start+c.count);for(let S=M,x=T-1;S<x;S+=m){const _=g.getX(S),P=g.getX(S+1),R=Fl(this,e,Ka,h,_,P);R&&t.push(R)}if(this.isLineLoop){const S=g.getX(T-1),x=g.getX(M),_=Fl(this,e,Ka,h,S,x);_&&t.push(_)}}else{const M=Math.max(0,c.start),T=Math.min(v.count,c.start+c.count);for(let S=M,x=T-1;S<x;S+=m){const _=Fl(this,e,Ka,h,S,S+1);_&&t.push(_)}if(this.isLineLoop){const S=Fl(this,e,Ka,h,T-1,M);S&&t.push(S)}}}updateMorphTargets(){const t=this.geometry.morphAttributes,r=Object.keys(t);if(r.length>0){const o=t[r[0]];if(o!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let u=0,c=o.length;u<c;u++){const d=o[u].name||String(u);this.morphTargetInfluences.push(0),this.morphTargetDictionary[d]=u}}}}}function Fl(s,e,t,r,o,u){const c=s.geometry.attributes.position;if(Jl.fromBufferAttribute(c,o),eu.fromBufferAttribute(c,u),t.distanceSqToSegment(Jl,eu,Sf,ug)>r)return;Sf.applyMatrix4(s.matrixWorld);const h=e.ray.origin.distanceTo(Sf);if(!(h<e.near||h>e.far))return{distance:h,point:ug.clone().applyMatrix4(s.matrixWorld),index:o,face:null,faceIndex:null,object:s}}const cg=new J,fg=new J;class a_ extends s_{constructor(e,t){super(e,t),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,r=[];for(let o=0,u=t.count;o<u;o+=2)cg.fromBufferAttribute(t,o),fg.fromBufferAttribute(t,o+1),r[o]=o===0?0:r[o-1],r[o+1]=r[o]+cg.distanceTo(fg);e.setAttribute("lineDistance",new _n(r,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class uw extends En{constructor(e,t,r,o,u,c,d,h,m){super(e,t,r,o,u,c,d,h,m),this.isCanvasTexture=!0,this.needsUpdate=!0}}class Ad extends Hn{constructor(e=1,t=1,r=1,o=32,u=1,c=!1,d=0,h=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:e,radiusBottom:t,height:r,radialSegments:o,heightSegments:u,openEnded:c,thetaStart:d,thetaLength:h};const m=this;o=Math.floor(o),u=Math.floor(u);const g=[],y=[],v=[],M=[];let T=0;const S=[],x=r/2;let _=0;P(),c===!1&&(e>0&&R(!0),t>0&&R(!1)),this.setIndex(g),this.setAttribute("position",new _n(y,3)),this.setAttribute("normal",new _n(v,3)),this.setAttribute("uv",new _n(M,2));function P(){const L=new J,$=new J;let O=0;const D=(t-e)/r;for(let j=0;j<=u;j++){const b=[],w=j/u,I=w*(t-e)+e;for(let Y=0;Y<=o;Y++){const K=Y/o,oe=K*h+d,ne=Math.sin(oe),B=Math.cos(oe);$.x=I*ne,$.y=-w*r+x,$.z=I*B,y.push($.x,$.y,$.z),L.set(ne,D,B).normalize(),v.push(L.x,L.y,L.z),M.push(K,1-w),b.push(T++)}S.push(b)}for(let j=0;j<o;j++)for(let b=0;b<u;b++){const w=S[b][j],I=S[b+1][j],Y=S[b+1][j+1],K=S[b][j+1];g.push(w,I,K),g.push(I,Y,K),O+=6}m.addGroup(_,O,0),_+=O}function R(L){const $=T,O=new ft,D=new J;let j=0;const b=L===!0?e:t,w=L===!0?1:-1;for(let Y=1;Y<=o;Y++)y.push(0,x*w,0),v.push(0,w,0),M.push(.5,.5),T++;const I=T;for(let Y=0;Y<=o;Y++){const oe=Y/o*h+d,ne=Math.cos(oe),B=Math.sin(oe);D.x=b*B,D.y=x*w,D.z=b*ne,y.push(D.x,D.y,D.z),v.push(0,w,0),O.x=ne*.5+.5,O.y=B*.5*w+.5,M.push(O.x,O.y),T++}for(let Y=0;Y<o;Y++){const K=$+Y,oe=I+Y;L===!0?g.push(oe,oe+1,K):g.push(oe+1,oe,K),j+=3}m.addGroup(_,j,L===!0?1:2),_+=j}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new Ad(e.radiusTop,e.radiusBottom,e.height,e.radialSegments,e.heightSegments,e.openEnded,e.thetaStart,e.thetaLength)}}class ou extends Ad{constructor(e=1,t=1,r=32,o=1,u=!1,c=0,d=Math.PI*2){super(0,e,t,r,o,u,c,d),this.type="ConeGeometry",this.parameters={radius:e,height:t,radialSegments:r,heightSegments:o,openEnded:u,thetaStart:c,thetaLength:d}}static fromJSON(e){return new ou(e.radius,e.height,e.radialSegments,e.heightSegments,e.openEnded,e.thetaStart,e.thetaLength)}}class Ol extends ns{constructor(e){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new _t(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new _t(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Og,this.normalScale=new ft(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yi,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}}class o_ extends Qt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new _t(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){const t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(t.object.target=this.target.uuid),t}}class cw extends o_{constructor(e,t,r){super(e,r),this.isHemisphereLight=!0,this.type="HemisphereLight",this.position.copy(Qt.DEFAULT_UP),this.updateMatrix(),this.groundColor=new _t(t)}copy(e,t){return super.copy(e,t),this.groundColor.copy(e.groundColor),this}}const Mf=new Vt,dg=new J,hg=new J;class fw{constructor(e){this.camera=e,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new ft(512,512),this.map=null,this.mapPass=null,this.matrix=new Vt,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new wd,this._frameExtents=new ft(1,1),this._viewportCount=1,this._viewports=[new Zt(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){const t=this.camera,r=this.matrix;dg.setFromMatrixPosition(e.matrixWorld),t.position.copy(dg),hg.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(hg),t.updateMatrixWorld(),Mf.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Mf),r.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),r.multiply(Mf)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.intensity=e.intensity,this.bias=e.bias,this.radius=e.radius,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const e={};return this.intensity!==1&&(e.intensity=this.intensity),this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}}class dw extends fw{constructor(){super(new qs(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class hw extends o_{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Qt.DEFAULT_UP),this.updateMatrix(),this.target=new Qt,this.shadow=new dw}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}}class pw{constructor(e=1,t=0,r=0){return this.radius=e,this.phi=t,this.theta=r,this}set(e,t,r){return this.radius=e,this.phi=t,this.theta=r,this}copy(e){return this.radius=e.radius,this.phi=e.phi,this.theta=e.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(e){return this.setFromCartesianCoords(e.x,e.y,e.z)}setFromCartesianCoords(e,t,r){return this.radius=Math.sqrt(e*e+t*t+r*r),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(e,r),this.phi=Math.acos(Mn(t/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}class mw extends a_{constructor(e=10,t=10,r=4473924,o=8947848){r=new _t(r),o=new _t(o);const u=t/2,c=e/t,d=e/2,h=[],m=[];for(let v=0,M=0,T=-d;v<=t;v++,T+=c){h.push(-d,0,T,d,0,T),h.push(T,0,-d,T,0,d);const S=v===u?r:o;S.toArray(m,M),M+=3,S.toArray(m,M),M+=3,S.toArray(m,M),M+=3,S.toArray(m,M),M+=3}const g=new Hn;g.setAttribute("position",new _n(h,3)),g.setAttribute("color",new _n(m,3));const y=new au({vertexColors:!0,toneMapped:!1});super(g,y),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}class gw extends a_{constructor(e=1){const t=[0,0,0,e,0,0,0,0,0,0,e,0,0,0,0,0,0,e],r=[1,0,0,1,.6,0,0,1,0,.6,1,0,0,0,1,0,.6,1],o=new Hn;o.setAttribute("position",new _n(t,3)),o.setAttribute("color",new _n(r,3));const u=new au({vertexColors:!0,toneMapped:!1});super(o,u),this.type="AxesHelper"}setColors(e,t,r){const o=new _t,u=this.geometry.attributes.color.array;return o.set(e),o.toArray(u,0),o.toArray(u,3),o.set(t),o.toArray(u,6),o.toArray(u,9),o.set(r),o.toArray(u,12),o.toArray(u,15),this.geometry.attributes.color.needsUpdate=!0,this}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:md}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=md);function Zn(s,e=1){return s==null||Number.isNaN(s)?"---":Number(s).toFixed(e)}function _w(s){return s==null||Number.isNaN(s)?0:Math.max(0,Math.min(1,s))}function Cd(s,e){return s==null||Number.isNaN(s)||!e?"ok":e.low_bad?e.alarm!==void 0&&s<=e.alarm?"alarm":e.warn!==void 0&&s<=e.warn?"warn":"ok":e.alarm!==void 0&&s>=e.alarm?"alarm":e.warn!==void 0&&s>=e.warn?"warn":"ok"}function Rd(s){return s==="alarm"?"#ff4d4f":s==="warn"?"#f5c542":"#3dd68c"}function bd(s,e){if(!e||s===null||s===void 0||Number.isNaN(s))return 0;const t=e.min??0,r=e.max??1;return r===t?0:_w((s-t)/(r-t))}function vw({attitude:s}){const e=it.useRef(null),t=it.useRef(null),r=it.useRef(null);it.useEffect(()=>{const u=t.current,c=e.current,d=new aw({canvas:u,antialias:!0,alpha:!1});d.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),d.setScissorTest(!0),d.setClearColor(658963,1);const h=new n_,m=new cw(10406399,1712684,1.1);h.add(m);const g=new hw(16777215,.6);g.position.set(2,3,1),h.add(g);const y=new Zr;y.rotation.x=-Math.PI/2,h.add(y);const v=Mw();v.add(new gw(.28)),y.add(v),h.add(new mw(1.6,8,2766146,1712684));const M=xw(),T=new qs(-1.15,1.15,1.15,-1.15,.1,10),S=new $n(42,1,.05,20),x=new J(.85,.55,.85),_=new pw().setFromVector3(x),P={theta:_.theta,phi:_.phi,radius:_.radius};wf(S,P);const R=.55,L=new qs(-R,R,R,-R,.05,20);L.position.set(0,1.4,0),L.up.set(0,0,-1),L.lookAt(0,0,0);const $=new qs(-R,R,R,-R,.05,20);$.position.set(0,0,1.4),$.lookAt(0,0,0);const O=new qs(-R,R,R,-R,.05,20);O.position.set(-1.4,0,0),O.lookAt(0,0,0);const D={on:!1,x:0,y:0};r.current={renderer:d,scene:h,vehicle:v,cameras:{persp:S,top:L,side:$,front:O},gizmo:M,gizmoCam:T,wrap:c,orbit:P,drag:D};const j=ne=>{Ef(ne,c)&&(D.on=!0,D.x=ne.clientX,D.y=ne.clientY,c.setPointerCapture(ne.pointerId),c.classList.add("orbiting"))},b=ne=>{if(c.style.cursor=Ef(ne,c)||D.on?"grab":"default",!D.on)return;const B=ne.clientX-D.x,G=ne.clientY-D.y;D.x=ne.clientX,D.y=ne.clientY,P.theta-=B*.008,P.phi=pg(P.phi-G*.008,.08,Math.PI-.08),wf(S,P)},w=ne=>{if(D.on){D.on=!1;try{c.releasePointerCapture(ne.pointerId)}catch{}c.classList.remove("orbiting")}},I=ne=>{Ef(ne,c)&&(ne.preventDefault(),P.radius=pg(P.radius*(ne.deltaY>0?1.08:.92),.45,4),wf(S,P))};c.addEventListener("pointerdown",j),c.addEventListener("pointermove",b),c.addEventListener("pointerup",w),c.addEventListener("pointercancel",w),c.addEventListener("wheel",I,{passive:!1});let Y=0;const K=()=>{Y=requestAnimationFrame(K),Ew(r.current)},oe=new ResizeObserver(()=>mg(r.current));return oe.observe(c),mg(r.current),K(),()=>{cancelAnimationFrame(Y),oe.disconnect(),c.removeEventListener("pointerdown",j),c.removeEventListener("pointermove",b),c.removeEventListener("pointerup",w),c.removeEventListener("pointercancel",w),c.removeEventListener("wheel",I),d.dispose()}},[]),it.useEffect(()=>{var m;const u=(m=r.current)==null?void 0:m.vehicle;if(!u||!s)return;const c=Tf(s.roll_deg),d=Tf(s.pitch_deg),h=Tf(s.yaw_deg);u.rotation.order="ZYX",u.rotation.set(c,d,h)},[s]);const o=`R ${Zn(s==null?void 0:s.roll_deg,1)}  P ${Zn(s==null?void 0:s.pitch_deg,1)}  Y ${Zn(s==null?void 0:s.yaw_deg,1)}`;return V.jsxs("div",{className:"card card-fill",children:[V.jsxs("h2",{children:["Attitude · IMU ",(s==null?void 0:s.source)==="snapshot"?"(10 Hz)":(s==null?void 0:s.source)==="monitor"?"(1 Hz)":""]}),V.jsxs("div",{ref:e,className:"attitude-grid",children:[V.jsx("canvas",{ref:t,style:{position:"absolute",inset:0,width:"100%",height:"100%"}}),V.jsx(kl,{title:"3D · drag",rpy:o}),V.jsx(kl,{title:"Top (yaw)",rpy:o}),V.jsx(kl,{title:"Side (pitch)",rpy:o}),V.jsx(kl,{title:"Front (roll)",rpy:o})]})]})}function kl({title:s,rpy:e}){return V.jsxs("div",{className:"view-box",style:{background:"transparent",pointerEvents:"none"},children:[V.jsx("div",{className:"view-label",children:s}),V.jsx("div",{className:"view-rpy",children:e})]})}function Ef(s,e){const t=e.getBoundingClientRect(),r=s.clientX-t.left,o=s.clientY-t.top;return r>=0&&o>=0&&r<t.width/2&&o<t.height/2}function wf(s,e){s.position.setFromSphericalCoords(e.radius,e.phi,e.theta),s.lookAt(0,0,0),s.updateProjectionMatrix()}function pg(s,e,t){return Math.max(e,Math.min(t,s))}function Tf(s){return s==null||Number.isNaN(s)?0:s*Math.PI/180}function xw(){const s=new n_,e=new Zr;e.rotation.x=-Math.PI/2,s.add(e);const t=.85,r=[{dir:[1,0,0],color:16731471,label:"X"},{dir:[0,1,0],color:4052620,label:"Y"},{dir:[0,0,1],color:5030911,label:"Z"}];for(const o of r){const u=new Hn().setFromPoints([new J(0,0,0),new J(o.dir[0]*t,o.dir[1]*t,o.dir[2]*t)]),c=new s_(u,new au({color:o.color,depthTest:!1,transparent:!0}));c.renderOrder=10,e.add(c);const d=new zn(new ou(.07,.18,8),new Ed({color:o.color,depthTest:!1}));d.position.set(o.dir[0]*t,o.dir[1]*t,o.dir[2]*t),o.dir[0]?d.rotation.z=-Math.PI/2:o.dir[1]||(d.rotation.x=Math.PI/2),d.renderOrder=11,e.add(d);const h=yw(o.label,o.color);h.position.set(o.dir[0]*(t+.22),o.dir[1]*(t+.22),o.dir[2]*(t+.22)),h.renderOrder=12,e.add(h)}return s}function yw(s,e){const t=document.createElement("canvas");t.width=64,t.height=64;const r=t.getContext("2d");r.clearRect(0,0,64,64),r.font="bold 44px IBM Plex Sans, sans-serif",r.textAlign="center",r.textBaseline="middle",r.lineWidth=6,r.strokeStyle="#070b10",r.strokeText(s,32,34),r.fillStyle=`#${e.toString(16).padStart(6,"0")}`,r.fillText(s,32,34);const o=new uw(t);o.colorSpace=li;const u=new i_({map:o,depthTest:!1,depthWrite:!1,transparent:!0}),c=new lw(u);return c.scale.set(.4,.4,.4),c}function Sw(s,e){const t=e.position.clone();t.lengthSq()<1e-8?t.set(0,0,1):t.normalize(),s.position.copy(t.multiplyScalar(2.4)),s.up.copy(e.up),s.lookAt(0,0,0),s.updateProjectionMatrix()}function Mw(){const s=new Zr,e=new zn(new ts(.42,.18,.14),new Ol({color:4034521,metalness:.2,roughness:.5}));s.add(e);const t=new zn(new ou(.07,.14,12),new Ol({color:16106818}));t.rotation.z=-Math.PI/2,t.position.x=.26,s.add(t);const r=new zn(new ts(.08,.04,.02),new Ol({color:16731471}));r.position.set(.05,.11,.04),s.add(r);const o=r.clone();return o.material=new Ol({color:4052620}),o.position.set(.05,-.11,.04),s.add(o),s}function mg(s){if(!s)return;const{wrap:e,renderer:t,cameras:r}=s,o=Math.max(1,e.clientWidth),u=Math.max(1,e.clientHeight);t.setSize(o,u,!1);const c=o/2/Math.max(1,u/2);r.persp.aspect=c,r.persp.updateProjectionMatrix()}function Ew(s){if(!s)return;const{renderer:e,scene:t,cameras:r,gizmo:o,gizmoCam:u,wrap:c}=s,d=c.clientWidth,h=c.clientHeight;if(d<2||h<2)return;const m=d/2,g=h/2,y=[{cam:r.persp,x:0,y:g,ww:m,hh:g},{cam:r.top,x:m,y:g,ww:m,hh:g},{cam:r.side,x:0,y:0,ww:m,hh:g},{cam:r.front,x:m,y:0,ww:m,hh:g}];for(const v of y){e.setViewport(v.x,v.y,v.ww,v.hh),e.setScissor(v.x,v.y,v.ww,v.hh),e.render(t,v.cam);const M=Math.max(36,Math.min(v.ww,v.hh)*.32),T=Math.max(4,M*.06),S=v.x+T,x=v.y+T;Sw(u,v.cam),e.clearDepth(),e.setViewport(S,x,M,M),e.setScissor(S,x,M,M),e.render(o,u)}}const l_="kankai.ui.v1";function Pd(){try{const s=localStorage.getItem(l_);if(!s)return{};const e=JSON.parse(s);return e&&typeof e=="object"?e:{}}catch{return{}}}function dd(s){try{const e={...Pd(),...s,updatedAt:Date.now()};return localStorage.setItem(l_,JSON.stringify(e)),e}catch{return null}}const Qr=9,ww=Array.from({length:Qr*Qr},(s,e)=>[e%Qr,Math.floor(e/Qr)]),Ld=.1,Ja=10;function gg(s){const e=Number(s)||0;return e<=.05?Ja:Math.min(Ja,Math.max(Ld,1/e))}function Tw(s){const e=Number(s);return!Number.isFinite(e)||e>=Ja-1e-6?0:1/Math.max(Ld,e)}function Aw(s,e){return(Number(e)||0)<=.05||s>=Ja-1e-6?"∞":Number(s).toFixed(2)}function Cw(s,e){const t=1/Qr,r=1/Qr;return[s*t,e*r,t,r].map(o=>Number(o.toFixed(4)))}function Rw(s){if(!s)return null;const e=String(s).split(",").map(Number);return e.length!==4||e.some(t=>Number.isNaN(t))?null:e}function bw(s,e){if(!s||!e)return!1;const t=.5/Qr;return s.every((r,o)=>Math.abs(r-e[o])<t)}function Pw(s){if(!s)return!0;const[e,t,r,o]=s;return e<=.02&&t<=.02&&e+r>=.98&&t+o>=.98}async function Lw(s,e){const t=await fetch(`/api/cameras/${encodeURIComponent(s)}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(e)}),r=await t.json().catch(()=>({}));if(!t.ok)throw new Error(r.error||`camera ${t.status}`);return r}function _g(s,e){const t=String(e||"").replace(/^\/+|\/+$/g,"");try{const r=new URL(s||"http://127.0.0.1:8889");return r.hostname=window.location.hostname||r.hostname,r.pathname=`/${t}/`,r.search="",r.hash="",r.toString()}catch{return`http://${window.location.hostname||"127.0.0.1"}:8889/${t}/`}}function Nw({config:s}){var ue,le,F,ce,Ie,te,fe,xe,Me,Le,ke,Ye,wt,z,bt,vt,yt,We,Lt,tt,rt,U,A,se,_e,ye,me,je,be,De,ot,Ee,Ne,pt,Je,Oe,st,lt,Tt,X,we,de,he,Te,qe;const e=(s==null?void 0:s.cameras)||{},t=Array.isArray(e.items)&&e.items.length?e.items:[{id:"cam0",path:"cam0",nickname:"Ceiling"},{id:"cam1",path:"cam1",nickname:"Canopy"}],r=Array.isArray(e.zoom)&&e.zoom.length?e.zoom:[{id:"binned",label:"1x Wide"},{id:"full",label:"1x Full res"},{id:"crop",label:"1.5x Crop"}],o=e.webrtc||"http://127.0.0.1:8889",[u,c]=it.useState(()=>Pd().camId||"cam0"),[d,h]=it.useState(null),[m,g]=it.useState(!1),[y,v]=it.useState(""),M=pe=>{c(pe),dd({camId:pe})};it.useEffect(()=>{var pe;if(!t.some(Ke=>Ke.id===u)){const Ke=((pe=t[0])==null?void 0:pe.id)||"cam0";c(Ke)}},[t,u]);const T=async()=>{try{const Ke=await(await fetch("/api/cameras")).json();h(Ke),Ke.error&&v(Ke.error)}catch(pe){v(String(pe.message||pe))}};it.useEffect(()=>{T();const pe=setInterval(T,2500);return()=>clearInterval(pe)},[]);const S=it.useMemo(()=>((d==null?void 0:d.state)||[]).find(pe=>pe.id===u)||{},[d,u]),x=new Set(Array.isArray(e.live)?e.live:["exposure","wb"]),_=pe=>!x.has(pe),P=pe=>_(pe)?"restart":"live",R=(d==null?void 0:d.backend)||e.backend||"mediamtx",L=(d==null?void 0:d.webrtc)||o,$=Rw((ue=S.focus)==null?void 0:ue.window),O=Pw($),D=((le=S.focus)==null?void 0:le.mode)==="continuous"||((F=S.focus)==null?void 0:F.mode)==="auto",j=((ce=S.exposure)==null?void 0:ce.mode)==="me"?"me":"ae",b=j==="ae",w=j==="me",I=((Ie=S.wb)==null?void 0:Ie.mode)==="manual",Y=!I,[K,oe]=it.useState({shutter_us:8e3,gain:2}),[ne,B]=it.useState([2,1.5]);it.useEffect(()=>{var mt,ut,Yt,qt;const pe=Number(((mt=S.exposure)==null?void 0:mt.seed_shutter_us)??((ut=S.exposure)==null?void 0:ut.shutter_us)),Ke=Number(((Yt=S.exposure)==null?void 0:Yt.seed_gain)??((qt=S.exposure)==null?void 0:qt.gain));Number.isFinite(pe)&&pe>0&&Number.isFinite(Ke)&&Ke>0&&oe({shutter_us:pe,gain:Ke})},[(te=S.exposure)==null?void 0:te.mode,(fe=S.exposure)==null?void 0:fe.shutter_us,(xe=S.exposure)==null?void 0:xe.gain,(Me=S.exposure)==null?void 0:Me.seed_shutter_us,(Le=S.exposure)==null?void 0:Le.seed_gain]),it.useEffect(()=>{var ut,Yt,qt;const pe=(Yt=(ut=S.wb)==null?void 0:ut.seed_gains)!=null&&Yt[0]?S.wb.seed_gains:(qt=S.wb)==null?void 0:qt.gains,Ke=Number(pe==null?void 0:pe[0]),mt=Number(pe==null?void 0:pe[1]);Number.isFinite(Ke)&&Ke>0&&Number.isFinite(mt)&&mt>0&&B([Ke,mt])},[(ke=S.wb)==null?void 0:ke.mode,(Ye=S.wb)==null?void 0:Ye.gains,(wt=S.wb)==null?void 0:wt.seed_gains]);const G=async(pe,Ke)=>{g(!0),v(Ke?"Restarting stream…":"Applying…");try{const mt=await Lw(u,pe);v(mt.restart?"Stream restarted":"Applied live"),await T()}catch(mt){v(String(mt.message||mt))}finally{g(!1)}},k=pe=>{const Ke=pe.path||pe.id;window.open(_g(L,Ke),"_blank","noopener,noreferrer")};return V.jsxs("div",{className:"card control-card cam-panel",children:[V.jsx("h2",{children:"Cameras"}),V.jsx("div",{className:"cam-notebook",role:"tablist","aria-label":"Cameras",children:t.map(pe=>{const Ke=(pe.nickname||pe.id||"").toLowerCase(),mt=Ke.includes("canopy")?"canopy":Ke.includes("ceil")?"ceiling":pe.id==="cam1"?"canopy":"ceiling";return V.jsx("button",{type:"button",role:"tab","aria-selected":pe.id===u,className:`cam-notebook-tab ${mt}${pe.id===u?" on":""}`,onClick:()=>M(pe.id),children:pe.nickname||pe.id},pe.id)})}),V.jsxs("div",{className:"cam-page",role:"tabpanel",children:[V.jsxs("div",{className:"cam-page-bar",children:[V.jsxs("span",{className:`lamp ${S.ready?"live":"stale"}`,children:[V.jsx("i",{})," ",S.ready?"ready":"idle"]}),V.jsx("span",{className:"cam-note",children:R}),V.jsx("a",{className:"btn cam-stream",href:_g(L,S.path||u),target:"_blank",rel:"noopener noreferrer",title:`Open WebRTC stream (${S.path||u})`,onClick:pe=>{pe.preventDefault();const Ke=t.find(mt=>mt.id===u)||{id:u,path:S.path||u};k(Ke)},children:"Stream"})]}),V.jsxs("div",{className:"cam-section",children:[V.jsxs("div",{className:"cam-label",children:["Zoom (sensor mode · ",P("zoom"),")"]}),V.jsx("div",{className:"cam-seg",children:r.map(pe=>V.jsx("button",{type:"button",disabled:m,className:S.zoom===pe.id?"btn primary":"btn",title:pe.hint||(pe.width&&pe.height?`Sensor ${pe.sensor_mode||"—"} → stream ${pe.width}x${pe.height} @ ${pe.fps||"—"} fps`:void 0),onClick:()=>G({zoom:pe.id},_("zoom")),children:pe.label},pe.id))})]}),V.jsxs("div",{className:"cam-grid",children:[V.jsxs("div",{className:"cam-col cam-col-focus",children:[V.jsxs("div",{className:"cam-label",children:["Focus (",P("focus"),")"]}),V.jsxs("div",{className:"cam-seg",children:[V.jsx("button",{type:"button",disabled:m,className:((z=S.focus)==null?void 0:z.mode)==="continuous"?"btn primary":"btn",onClick:()=>G({focus:{mode:"continuous"}},_("focus")),children:"AF cont."}),V.jsx("button",{type:"button",disabled:m,className:((bt=S.focus)==null?void 0:bt.mode)==="auto"?"btn primary":"btn",onClick:()=>G({focus:{mode:"auto"}},_("focus")),children:"AF once"}),V.jsx("button",{type:"button",disabled:m,className:((vt=S.focus)==null?void 0:vt.mode)==="manual"?"btn primary":"btn",onClick:()=>{var pe;return G({focus:{mode:"manual",lens_position:((pe=S.focus)==null?void 0:pe.lens_position)||1}},_("focus"))},title:"Manual focus — set focus distance",children:"MF"})]}),V.jsxs("div",{className:"cam-label",children:["AF window",D?O?" · full frame":" · region":""]}),V.jsx("div",{className:`af-grid${D?" af-mode":""}${O?" af-full":""}`,children:ww.map(([pe,Ke])=>{const mt=Cw(pe,Ke),ut=!O&&bw($,mt);return V.jsx("button",{type:"button",disabled:m,className:ut?"af-cell on":"af-cell","aria-pressed":ut,title:`AF cell ${pe+1},${Ke+1}`,onClick:()=>G({focus:{window:mt}},_("focus"))},`${pe}-${Ke}`)})}),V.jsx("button",{type:"button",className:`btn cam-wide${O?" primary":""}`,disabled:m,"aria-pressed":O,onClick:()=>G({focus:{window:""}},_("focus")),children:"Full frame"}),((yt=S.focus)==null?void 0:yt.mode)==="manual"&&V.jsx(Ys,{label:"Focus",unit:"m",display:Aw(gg((We=S.focus)==null?void 0:We.lens_position),(Lt=S.focus)==null?void 0:Lt.lens_position),digits:2,min:Ld,max:Ja,step:.05,disabled:m,value:gg((tt=S.focus)==null?void 0:tt.lens_position),onCommit:pe=>G({focus:{mode:"manual",lens_position:Tw(pe)}},_("focus"))})]}),V.jsxs("div",{className:"cam-col cam-col-exp",children:[V.jsxs("div",{className:"cam-label",children:["Exposure (",w?"manual":"auto"," · ",P("exposure"),")"]}),V.jsxs("div",{className:"cam-seg",children:[V.jsx("button",{type:"button",disabled:m,className:b?"btn primary":"btn",onClick:()=>G({exposure:{mode:"ae"}},!0),children:"Auto"}),V.jsx("button",{type:"button",disabled:m,className:w?"btn primary":"btn",onClick:()=>G({exposure:{mode:"me"}},_("exposure")),children:"Manual"})]}),V.jsx(Ys,{label:"EV",digits:1,min:-4,max:4,step:.5,disabled:m||!b,value:Number(((rt=S.exposure)==null?void 0:rt.ev)??0),onCommit:pe=>G({exposure:{mode:"ae",ev:pe}},_("exposure"))}),V.jsx(Ys,{label:"Shutter",unit:"µs",digits:0,min:100,max:33e3,step:100,disabled:m||!w,value:Number(w?((U=S.exposure)==null?void 0:U.shutter_us)??((A=S.exposure)==null?void 0:A.seed_shutter_us)??K.shutter_us:((se=S.exposure)==null?void 0:se.seed_shutter_us)??((_e=S.exposure)==null?void 0:_e.shutter_us)??K.shutter_us),onCommit:pe=>G({exposure:{mode:"me",shutter_us:pe,gain:K.gain}},_("exposure"))}),V.jsx(Ys,{label:"Gain",digits:1,min:1,max:12,step:.1,disabled:m||!w,value:Number(w?((ye=S.exposure)==null?void 0:ye.gain)??((me=S.exposure)==null?void 0:me.seed_gain)??K.gain:((je=S.exposure)==null?void 0:je.seed_gain)??((be=S.exposure)==null?void 0:be.gain)??K.gain),onCommit:pe=>G({exposure:{mode:"me",shutter_us:K.shutter_us,gain:pe}},_("exposure"))}),V.jsxs("div",{className:"cam-label",children:["White balance (",I?"manual":"auto"," · ",P("wb"),")"]}),V.jsxs("div",{className:"cam-seg",children:[V.jsx("button",{type:"button",disabled:m,className:Y?"btn primary":"btn",onClick:()=>G({wb:{mode:"auto"}},_("wb")),children:"Auto"}),V.jsx("button",{type:"button",disabled:m,className:I?"btn primary":"btn",onClick:()=>G({wb:{mode:"manual"}},_("wb")),children:"Manual"})]}),V.jsx(Ys,{label:"R",digits:2,min:.5,max:8,step:.05,disabled:m||!I,value:Number(I?((ot=(De=S.wb)==null?void 0:De.gains)==null?void 0:ot[0])??((Ne=(Ee=S.wb)==null?void 0:Ee.seed_gains)==null?void 0:Ne[0])??ne[0]:((Je=(pt=S.wb)==null?void 0:pt.seed_gains)==null?void 0:Je[0])??((st=(Oe=S.wb)==null?void 0:Oe.gains)==null?void 0:st[0])??ne[0]),onCommit:pe=>G({wb:{mode:"manual",gains:[pe,ne[1]]}},_("wb"))}),V.jsx(Ys,{label:"B",digits:2,min:.5,max:8,step:.05,disabled:m||!I,value:Number(I?((Tt=(lt=S.wb)==null?void 0:lt.gains)==null?void 0:Tt[1])??((we=(X=S.wb)==null?void 0:X.seed_gains)==null?void 0:we[1])??ne[1]:((he=(de=S.wb)==null?void 0:de.seed_gains)==null?void 0:he[1])??((qe=(Te=S.wb)==null?void 0:Te.gains)==null?void 0:qe[1])??ne[1]),onCommit:pe=>G({wb:{mode:"manual",gains:[ne[0],pe]}},_("wb"))})]})]}),V.jsx("div",{className:"cam-note",children:y||`${S.width||"—"}x${S.height||"—"} @ ${S.fps||"—"} fps`})]})]})}function Ys({label:s,value:e,min:t,max:r,step:o,digits:u,unit:c="",display:d,disabled:h,onCommit:m}){const[g,y]=it.useState(e);it.useEffect(()=>{y(e)},[e]);const v=d!=null?g===e?d:Number(g)>=r-1e-6?"∞":Number(g).toFixed(u):Number(g).toFixed(u);return V.jsxs("label",{className:`cam-slider${h?" disabled":""}`,children:[s," ",v,c&&v!=="∞"?` ${c}`:"",V.jsx("input",{type:"range",min:t,max:r,step:o,disabled:h,value:g,onChange:M=>{h||y(Number(M.target.value))},onPointerUp:M=>{h||m(Number(M.currentTarget.value))},onKeyUp:M=>{h||(M.key==="Enter"||M.key==="ArrowLeft"||M.key==="ArrowRight")&&m(Number(M.currentTarget.value))}})]})}const Dw=[{key:"x",label:"X"},{key:"y",label:"Y"},{key:"z",label:"Z"},{key:"yaw",label:"Yaw"},{key:"pitch",label:"Pitch"},{key:"roll",label:"Roll"}],Iw=[{key:"grab",label:"Grip"},{key:"roll",label:"Roll"}],vg={x:0,y:0,z:0,yaw:0,pitch:0,roll:0},hd={grab:0,roll:0};function gn(s,e,t){return Math.max(e,Math.min(t,s))}function Af(s){return gn(s,-100,100)/100}function Cf(s,e=-100,t=100){return gn((Number(s)||0)*100,e,t)}function Rf(s){return Math.round(s)}function xg(s,e){for(const t of Object.keys(e))if(s[t]!==e[t])return e;return s}function yg(){const e=Pd().hand;return!e||typeof e!="object"?hd:{grab:gn(Number(e.grab)||0,-1,1),roll:gn(Number(e.roll)||0,-1,1)}}async function Bl(s){const e=await fetch("/api/command",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(s)});if(!e.ok)throw new Error(`command ${e.status}`);return e.json()}function Uw({command:s,lights:e,stepPercent:t=5,children:r}){const o=Array.isArray(e)&&e.length?e:[],[u,c]=it.useState(vg),[d,h]=it.useState(yg),[m,g]=it.useState(()=>Object.fromEntries(o.map(B=>[B,0]))),[y,v]=it.useState(()=>Object.fromEntries(o.map(B=>[B,!1]))),M=it.useRef({twist:!1,hand:!1,lights:!1}),T=it.useRef({twist:!1,hand:!1,lights:!1}),S=it.useRef({twist:!1,hand:!1,lights:!1}),x=it.useRef({twist:null,hand:null,lights:null}),_=it.useRef(y);_.current=y;const P=it.useRef({twist:u,hand:d,lights:m});P.current={twist:u,hand:d,lights:m};const R=B=>{!T.current[B]&&!S.current[B]&&x.current[B]==null&&(M.current[B]=!1)},L=(B,G)=>{const k={};for(const ue of o)k[ue]=G[ue]?B[ue]??0:0;return k};it.useEffect(()=>{g(B=>{const G={};for(const k of o)G[k]=B[k]??0;return G}),v(B=>{const G={};for(const k of o)G[k]=!!B[k];return G})},[e]),it.useEffect(()=>{s&&(!M.current.twist&&s.twist&&c(B=>xg(B,{x:gn(Number(s.twist.x)||0,-1,1),y:gn(Number(s.twist.y)||0,-1,1),z:gn(Number(s.twist.z)||0,-1,1),yaw:gn(Number(s.twist.yaw)||0,-1,1),pitch:gn(Number(s.twist.pitch)||0,-1,1),roll:gn(Number(s.twist.roll)||0,-1,1)})),!M.current.hand&&s.hand&&h(B=>xg(B,{grab:gn(Number(s.hand.grab)||0,-1,1),roll:gn(Number(s.hand.roll)||0,-1,1)})),!M.current.lights&&s.lights&&(g(B=>{const G={...B};let k=!1;for(const ue of o){if(s.lights[ue]==null)continue;const le=gn(Number(s.lights[ue])||0,0,1);le>0&&G[ue]!==le&&(G[ue]=le,k=!0)}return k?G:B}),v(B=>{const G={...B};let k=!1;for(const ue of o){if(s.lights[ue]==null)continue;const le=(Number(s.lights[ue])||0)>.001;!!G[ue]!==le&&(G[ue]=le,k=!0)}return k?G:B})))},[s,e,o]);const $=it.useCallback((B,G)=>{if(M.current[B]=!0,x.current[B]=G,S.current[B])return;S.current[B]=!0,(async()=>{for(;x.current[B]!=null;){const ue=x.current[B];x.current[B]=null;try{B==="twist"?await Bl({twist:ue}):B==="hand"?await Bl({hand:ue}):await Bl({lights:ue})}catch{}}S.current[B]=!1,R(B)})()},[]),O=(B,G)=>{const k={...P.current.twist,[B]:Af(G)};c(k),$("twist",k)},D=(B,G)=>{const k={...P.current.hand,[B]:Af(G)};h(k),dd({hand:k}),$("hand",k)},j=(B,G)=>{const k=Af(G),ue={...P.current.lights,[B]:k};g(ue),P.current.lights=ue;const le={..._.current};$("lights",L(ue,le))},b=B=>{const G={..._.current,[B]:!1};_.current=G,v(G),$("lights",L(P.current.lights,G))},w=B=>{const G={...P.current.lights};G[B]>0||(G[B]=1);const k={..._.current,[B]:!0};_.current=k,g(G),P.current.lights=G,v(k),$("lights",L(G,k))},I=()=>{M.current.twist=!0,M.current.hand=!0,T.current.twist=!1,T.current.hand=!1,x.current.twist=null,x.current.hand=null,c(vg),h(hd),dd({hand:hd}),S.current.twist=!0,S.current.hand=!0,Bl({estop:!0}).finally(()=>{S.current.twist=!1,S.current.hand=!1,R("twist"),R("hand")})},Y=()=>{const B=Object.fromEntries(o.map(G=>[G,!1]));_.current=B,v(B),T.current.lights=!1,x.current.lights=null,$("lights",L(P.current.lights,B))},K=()=>{const B=Object.fromEntries(o.map(k=>[k,1])),G=Object.fromEntries(o.map(k=>[k,!0]));_.current=G,g(B),P.current.lights=B,v(G),T.current.lights=!1,x.current.lights=null,$("lights",L(B,G))},oe=(B,G)=>{T.current[B]=G,G?M.current[B]=!0:R(B)},ne=it.useRef(!1);return it.useEffect(()=>{if(ne.current)return;ne.current=!0;const B=yg();(B.grab!==0||B.roll!==0)&&$("hand",B)},[$]),V.jsxs("div",{className:"control-dock",children:[V.jsxs("div",{className:"control-top",children:[V.jsxs("div",{className:"card control-card",children:[V.jsx("h2",{children:"Thrusters"}),V.jsx("div",{className:"fader-row",children:Dw.map(B=>V.jsx(bf,{label:B.label,value:Cf(u[B.key]),min:-100,max:100,step:t,bipolar:!0,onChange:G=>O(B.key,G),onActive:G=>oe("twist",G)},B.key))})]}),V.jsxs("div",{className:"card control-card",children:[V.jsx("h2",{children:"Manipulators"}),V.jsx("div",{className:"fader-row",children:Iw.map(B=>V.jsx(bf,{label:B.label,value:Cf(d[B.key]),min:-100,max:100,step:t,bipolar:!0,onChange:G=>D(B.key,G),onActive:G=>oe("hand",G)},B.key))})]}),V.jsxs("button",{className:"estop",type:"button",onClick:I,children:[V.jsx("span",{children:"E-STOP"}),V.jsx("b",{children:"N"})]})]}),V.jsxs("div",{className:"control-lights-wrap",children:[V.jsxs("div",{className:"card control-card control-lights",children:[V.jsx("h2",{children:"Lights"}),V.jsx("div",{className:"fader-row",children:o.map(B=>V.jsx(bf,{label:B,value:Cf(m[B]??0,0,100),min:0,max:100,step:t,lightSwitch:!0,lit:!!y[B],onChange:G=>j(B,G),onOff:()=>b(B),onOn:()=>w(B),onActive:G=>oe("lights",G)},B))})]}),V.jsxs("div",{className:"lights-bulk",children:[V.jsxs("button",{className:"lights-off",type:"button",onClick:Y,children:[V.jsx("span",{children:"All off"}),V.jsx("b",{children:"0"})]}),V.jsxs("button",{className:"lights-on",type:"button",onClick:K,children:[V.jsx("span",{children:"All on"}),V.jsx("b",{children:"100"})]})]})]}),r]})}function bf({label:s,value:e,min:t,max:r,step:o,bipolar:u=!1,lightSwitch:c=!1,lit:d=!1,onChange:h,onOff:m,onOn:g,onActive:y}){const v=it.useRef(null),M=it.useRef(null),T=Rf(gn(e,t,r)),S=r-t||1,x=(T-t)/S,_=u?(0-t)/S:0,P=(1-Math.max(x,_))*100,R=Math.min(x,_)*100,L=(1-x)*100,$=Y=>{const K=v.current;if(!K)return;const oe=K.getBoundingClientRect(),ne=gn((Y-oe.top)/oe.height,0,1);h(Rf(r-ne*S))},O=Y=>{h(Rf(gn(T+Y*o,t,r)))},D=Y=>{y==null||y(!0),O(Y),j(),M.current=setTimeout(()=>{M.current=setInterval(()=>O(Y),80)},380)},j=()=>{M.current&&(clearTimeout(M.current),clearInterval(M.current),M.current=null)},b=Y=>{Y.preventDefault(),y==null||y(!0),Y.currentTarget.setPointerCapture(Y.pointerId),$(Y.clientY)},w=Y=>{Y.currentTarget.hasPointerCapture(Y.pointerId)&&$(Y.clientY)},I=()=>{j(),y==null||y(!1)};return V.jsxs("div",{className:`fader${c&&d?" lit":""}`,children:[V.jsxs("div",{className:"fader-val",children:[T,"%"]}),V.jsx("button",{type:"button",className:"fader-btn","aria-label":`${s} plus`,onPointerDown:Y=>{Y.preventDefault(),Y.currentTarget.setPointerCapture(Y.pointerId),D(1)},onPointerUp:I,onPointerCancel:I,children:"+"}),V.jsxs("div",{className:"fader-track",ref:v,role:"slider","aria-label":s,"aria-valuemin":t,"aria-valuemax":r,"aria-valuenow":T,onPointerDown:b,onPointerMove:w,onPointerUp:I,onPointerCancel:I,children:[u&&V.jsx("i",{className:"fader-zero"}),V.jsx("div",{className:"fader-fill",style:{top:`${P}%`,bottom:`${R}%`}}),V.jsx("div",{className:"fader-knob",style:{top:`${L}%`}})]}),V.jsx("button",{type:"button",className:"fader-btn","aria-label":`${s} minus`,onPointerDown:Y=>{Y.preventDefault(),Y.currentTarget.setPointerCapture(Y.pointerId),D(-1)},onPointerUp:I,onPointerCancel:I,children:"−"}),c?V.jsxs("div",{className:"fader-switch",children:[V.jsx("button",{type:"button",className:`fader-on-btn${d?" active":""}`,"aria-label":`${s} on`,onClick:()=>{y==null||y(!0),g==null||g(),y==null||y(!1)},children:"On"}),V.jsx("button",{type:"button",className:`fader-off-btn${d?"":" active"}`,"aria-label":`${s} off`,onClick:()=>{y==null||y(!0),m==null||m(),y==null||y(!1)},children:"Off"})]}):V.jsx("button",{type:"button",className:"fader-zero-btn","aria-label":`${s} neutral`,onClick:()=>{y==null||y(!0),h(0),y==null||y(!1)},children:"N"}),V.jsx("div",{className:"fader-label",children:s})]})}const u_=[0,.25,.5,.75,1];function Nd(s,e){const t=(s==null?void 0:s.min)??0,r=(s==null?void 0:s.max)??1;return t+e*(r-t)}function c_(s,e){return 0}function _i({label:s,unit:e,value:t,spec:r,digits:o=1}){const u=Cd(t,r),c=t==null?"#8b9bb0":Rd(u),d=bd(t,r),h=Math.PI*.75,m=Math.PI*2.25,g=h+d*(m-h),y=80,v=68,M=48,T=Ks(y,v,M-8,g),S=Ow(y,v,M,h,m),x=c_(),_=u_.map(P=>{const R=h+P*(m-h),L=Ks(y,v,M+2,R),$=Ks(y,v,M-9,R),O=Ks(y,v,M+15,R);return{outer:L,inner:$,labelPos:O,major:P===0||P===.5||P===1,text:Zn(Nd(r,P),x)}});return V.jsxs("div",{className:"meter",children:[V.jsxs("svg",{viewBox:"0 0 160 140","aria-label":s,children:[V.jsx("path",{d:S,fill:"none",stroke:"#0a0e13",strokeWidth:"12",strokeLinecap:"round"}),V.jsx("path",{d:S,fill:"none",stroke:c,strokeWidth:"12",strokeLinecap:"round",strokeDasharray:`${d*Mg(M,h,m)} ${Mg(M,h,m)}`}),_.map((P,R)=>V.jsxs("g",{children:[V.jsx("line",{x1:P.inner.x,y1:P.inner.y,x2:P.outer.x,y2:P.outer.y,stroke:"#5a6a7a",strokeWidth:P.major?1.6:1}),P.major&&V.jsx("text",{x:P.labelPos.x,y:P.labelPos.y,textAnchor:"middle",dominantBaseline:"middle",className:"tick-label",fill:"#8b9bb0",fontSize:"11",children:P.text})]},R)),V.jsx("line",{x1:y,y1:v,x2:T.x,y2:T.y,stroke:"#f4f7fb",strokeWidth:"2.5"}),V.jsx("circle",{cx:y,cy:v,r:"4",fill:"#f4f7fb"})]}),V.jsxs("div",{className:"val",style:{color:c},children:[Zn(t,o),V.jsx("span",{className:"unit",children:e})]}),V.jsx("div",{className:"lbl",children:s})]})}function Sg({label:s,unit:e,value:t,spec:r,digits:o=1}){const u=Cd(t,r),c=t==null?"#8b9bb0":Rd(u),d=`${bd(t,r)*100}%`,h=c_(),m=u_.map(g=>({p:g,text:Zn(Nd(r,g),h),major:g===0||g===.5||g===1}));return V.jsxs("div",{className:"bar",children:[V.jsx("div",{children:s}),V.jsxs("div",{className:"bar-scale",children:[V.jsxs("div",{className:"track","aria-label":s,children:[V.jsxs("div",{className:"bar-ticks","aria-hidden":"true",children:[V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{})]}),V.jsx("div",{className:"fill",style:{width:d,background:c}})]}),V.jsx("div",{className:"bar-tick-labels","aria-hidden":"true",children:m.map((g,y)=>V.jsx("span",{className:g.major?"major":"",style:{left:`${g.p*100}%`},children:g.major?g.text:""},y))})]}),V.jsxs("div",{style:{color:c,textAlign:"right"},children:[Zn(t,o)," ",e]})]})}function Fw({nickname:s,value:e,spec:t}){const r=Cd(e,t),o=e==null?"#8b9bb0":Rd(r),u=`${bd(e,t)*100}%`,c=0,d=[0,.5,1].map(h=>({p:h,text:Zn(Nd(t,h),c)}));return V.jsxs("div",{className:"thermo",children:[V.jsxs("div",{className:"thermo-body",children:[V.jsx("div",{className:"thermo-tick-labels","aria-hidden":"true",children:d.map(h=>V.jsx("span",{style:{bottom:`${h.p*100}%`},children:h.text},h.p))}),V.jsxs("div",{className:"well","aria-label":s,children:[V.jsxs("div",{className:"thermo-ticks","aria-hidden":"true",children:[V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{}),V.jsx("i",{})]}),V.jsx("div",{className:"fill",style:{height:u,background:o}})]})]}),V.jsxs("div",{className:"thermo-caption",children:[V.jsx("div",{className:"name",children:s}),V.jsxs("div",{className:"val",style:{color:o},children:[Zn(e,1)," °C"]})]})]})}function Ks(s,e,t,r){return{x:s+t*Math.cos(r),y:e+t*Math.sin(r)}}function Ow(s,e,t,r,o){const u=Ks(s,e,t,r),c=Ks(s,e,t,o),d=o-r>Math.PI?1:0;return`M ${u.x} ${u.y} A ${t} ${t} 0 ${d} 1 ${c.x} ${c.y}`}function Mg(s,e,t){return s*(t-e)}const kw={monitor:null,attitude:null,health:{publisher:"never",imu:"never"}};function Bw(){var b;const[s,e]=it.useState(null),[t,r]=it.useState(kw),[o,u]=it.useState("connecting"),[c,d]=it.useState(!0),[h,m]=it.useState(!1),[g,y]=it.useState(!1),v=it.useRef(null),M=it.useRef({leak:!1,publisher:"never",stream:"connecting"}),T=it.useRef(null);it.useEffect(()=>{const w=cm.loadPrefs();m(w.quietAdvisories);const I=new cm;I.armed=!0,I.quietAdvisories=w.quietAdvisories,v.current=I,I.arm().then(()=>d(!0)).catch(()=>{});const Y=()=>{I.arm().then(()=>{d(!0),I.update(M.current)}).catch(()=>{})};return window.addEventListener("pointerdown",Y,{once:!0}),window.addEventListener("keydown",Y,{once:!0}),fetch("/api/config").then(K=>K.json()).then(e).catch(()=>e({temperatures:[],gauges:{}})),()=>{window.removeEventListener("pointerdown",Y),window.removeEventListener("keydown",Y)}},[]),it.useEffect(()=>{const w=new EventSource("/api/stream");let I=null;const Y=()=>{I&&(clearTimeout(I),I=null),u("connected")};return w.addEventListener("state",K=>{Y();try{const oe=JSON.parse(K.data);r(oe),oe.config&&e(oe.config)}catch{}}),w.onerror=()=>{I||(I=setTimeout(()=>{I=null,u("disconnected")},3500))},()=>{I&&clearTimeout(I),w.close()}},[]);const S=t.monitor,x=t.health||{},_=!!(S!=null&&S.water_ch0_detected||S!=null&&S.water_ch1_detected||x.leak),P=(s==null?void 0:s.gauges)||{},R=(s==null?void 0:s.temperatures)||[],L=(s==null?void 0:s.temperature_gauge)||{min:0,max:80},$=(s==null?void 0:s.leaks)||[];M.current={leak:_,publisher:x.publisher,stream:o};const O=_?{cls:"alarm",text:"Siren · water leak"}:h?{cls:"quiet",text:"Advisories off (siren on)"}:o==="disconnected"?{cls:"stale",text:"Pulse · stream lost (~5s)"}:x.publisher==="stale"||x.publisher==="never"?{cls:"stale",text:"Beep · publisher stale (~20s)"}:{cls:"quiet",text:"Alerts quiet"};it.useEffect(()=>{var w;(w=v.current)==null||w.update({leak:_,publisher:x.publisher,stream:o})},[_,x.publisher,o,c,h]);const D=()=>{var I;const w=!h;m(w),(I=v.current)==null||I.setQuietAdvisories(w)};it.useEffect(()=>{const w=()=>y(!!document.fullscreenElement);return document.addEventListener("fullscreenchange",w),()=>document.removeEventListener("fullscreenchange",w)},[]);const j=async()=>{var w;try{if(document.fullscreenElement)await document.exitFullscreen();else{const I=T.current||document.documentElement;await((w=I.requestFullscreen)==null?void 0:w.call(I))}}catch{}};return V.jsxs("div",{className:"app",ref:T,children:[V.jsxs("header",{className:"header",children:[V.jsx("div",{className:"brand",children:"Kankai Control Panel"}),V.jsx(Pf,{name:"Link",status:o==="connected"?"live":o}),V.jsx(Pf,{name:"Publisher",status:x.publisher||"never"}),V.jsx(Pf,{name:"IMU",status:x.imu||"never"}),V.jsxs("div",{className:"meta",children:[V.jsx("span",{children:(S==null?void 0:S.stamp_jst)||"---"}),V.jsxs("span",{children:["seq ",(S==null?void 0:S.seq)??"---"]}),V.jsxs("span",{children:["elapsed ",(S==null?void 0:S.elapsed_hms)||"---"]})]}),V.jsxs("div",{className:"header-actions",children:[V.jsxs("span",{className:`lamp ${O.cls}`,title:"Siren = water leak (never muted). Soft pulse ~5s = data stream lost. Soft beep ~20s = publisher stale.",children:[V.jsx("i",{})," ",O.text]}),V.jsx("button",{className:h?"btn":"btn armed",onClick:D,title:"Turn off advisory pulse/beep only. Leak siren always stays on.",children:h?"Advisories off":"Advisories on"}),V.jsx("button",{className:"btn",type:"button",onClick:j,children:g?"Exit full":"Fullscreen"})]})]}),o==="disconnected"&&V.jsx("div",{className:"banner",children:"Data stream disconnected — check publisher / node"}),o==="connected"&&(x.publisher==="stale"||x.publisher==="never")&&V.jsx("div",{className:"banner",children:"rov/monitor_value stalled — check monitor_value_pub"}),V.jsxs("div",{className:"layout",children:[V.jsxs("div",{className:"col",children:[V.jsx(Hw,{monitor:S,leak:_,leaks:$}),V.jsxs("div",{className:"card",children:[V.jsx("h2",{children:"Depth"}),V.jsxs("div",{className:"gauge-row",children:[V.jsx(_i,{label:"Depth (seawater)",unit:"m",value:S==null?void 0:S.depth_m,spec:P.depth_m,digits:2}),V.jsx(_i,{label:"Raw",unit:"atm",value:S==null?void 0:S.depth_pressure_atm,spec:P.depth_pressure_atm,digits:3}),V.jsx(_i,{label:"Temp",unit:"°C",value:S==null?void 0:S.depth_temp_c,spec:L,digits:1})]})]}),V.jsxs("div",{className:"card card-fill",children:[V.jsx("h2",{children:"Temperature"}),V.jsx("div",{className:"thermo-row",children:R.filter(w=>w.key!=="depth_temp_c").map(w=>V.jsx(Fw,{nickname:w.nickname||w.key,value:S?S[w.key]:null,spec:L},w.key))})]}),V.jsxs("div",{className:"card",children:[V.jsx("h2",{children:"Hull interior"}),V.jsxs("div",{className:"bars",children:[V.jsx(Sg,{label:"Humidity",unit:"%",value:S==null?void 0:S.bme_humidity_percent,spec:P.bme_humidity_percent}),V.jsx(Sg,{label:"Pressure",unit:"atm",value:S==null?void 0:S.bme_pressure_atm,spec:P.bme_pressure_atm,digits:3})]})]})]}),V.jsxs("div",{className:"col col-right",children:[V.jsx(vw,{attitude:t.attitude}),V.jsxs("div",{className:"card card-compact",children:[V.jsx("h2",{children:"Power"}),V.jsxs("div",{className:"gauge-row gauge-row-4",children:[V.jsx(_i,{label:"Voltage",unit:"V",value:S==null?void 0:S.voltage_v,spec:P.voltage_v,digits:1}),V.jsx(_i,{label:"Current",unit:"A",value:S==null?void 0:S.current_a,spec:P.current_a,digits:2}),V.jsx(_i,{label:"Power",unit:"W",value:S==null?void 0:S.power_w,spec:P.power_w,digits:1}),V.jsx(_i,{label:"Remaining",unit:"%",value:S==null?void 0:S.remaining_percent,spec:P.remaining_percent,digits:0})]}),V.jsxs("div",{className:"kv",style:{marginTop:8},children:[V.jsx("span",{children:"Energy"}),V.jsxs("b",{children:[Zn(S==null?void 0:S.accumulated_energy_wh,2)," Wh"]}),V.jsx("span",{children:"Peak"}),V.jsxs("b",{children:[Zn(S==null?void 0:S.peak_power_w,1)," W"]})]})]}),V.jsxs("div",{className:"card card-compact",children:[V.jsx("h2",{children:"RPi"}),V.jsxs("div",{className:"gauge-row",children:[V.jsx(_i,{label:"CPU",unit:"%",value:S==null?void 0:S.rpi_cpu_util_percent,spec:P.rpi_cpu_util_percent,digits:1}),V.jsx(_i,{label:"GPU",unit:"%",value:S==null?void 0:S.rpi_gpu_util_percent,spec:P.rpi_gpu_util_percent,digits:1}),V.jsx(_i,{label:"Fan",unit:"rpm",value:S==null?void 0:S.rpi_fan_rpm,spec:P.rpi_fan_rpm,digits:0})]})]})]}),V.jsx(Uw,{command:t.command,lights:s==null?void 0:s.lights,stepPercent:((b=s==null?void 0:s.command)==null?void 0:b.step_percent)??5,children:V.jsx(Nw,{config:s})})]})]})}function Pf({name:s,status:e}){const t=e==="live"?"live":e==="stale"||e==="reconnecting"||e==="connecting"?"stale":"disconnected";return V.jsxs("span",{className:`lamp ${t}`,children:[V.jsx("i",{})," ",s," ",zw(e)]})}function zw(s){return s==="live"?"live":s==="stale"?"stale":s==="never"?"none":s==="connecting"?"connecting":s==="disconnected"?"disconnected":s}function Hw({monitor:s,leak:e,leaks:t}){const r=Object.fromEntries((t||[]).map(o=>[Number(o.channel),o.nickname]));return V.jsxs("div",{className:e?"card alarm":"card",children:[V.jsx("h2",{children:"Water Leakage"}),V.jsxs("div",{className:"leak-grid",children:[V.jsx(Eg,{n:r[0]||"ch0",v:s==null?void 0:s.water_ch0_probe_v,hot:s==null?void 0:s.water_ch0_detected}),V.jsx(Eg,{n:r[1]||"ch1",v:s==null?void 0:s.water_ch1_probe_v,hot:s==null?void 0:s.water_ch1_detected})]})]})}function Eg({n:s,v:e,hot:t}){return V.jsxs("div",{className:t?"ch hot blink":"ch",children:[V.jsx("div",{className:"name",children:s}),V.jsxs("div",{className:"big",children:[Zn(e,2)," V"]}),V.jsx("div",{className:"status",children:t?"DETECTED":"ok"})]})}e0.createRoot(document.getElementById("root")).render(V.jsx(qv.StrictMode,{children:V.jsx(Bw,{})}));
