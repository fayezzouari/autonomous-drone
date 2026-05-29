import { forwardRef, useImperativeHandle, useMemo, useRef } from "react";
import * as THREE from "three";
import { COLOR, ROTOR_RADIUS } from "../consts";

// Handles the parent uses to animate the airframe each frame.
export interface CopterRefs {
  prop: THREE.Group | null;
  // Four independent vane hinges. v1/v3 on the X arm (fore/aft), v2/v4 on Y (lateral).
  vanes: (THREE.Group | null)[];
  vaneMats: THREE.MeshStandardMaterial[];
  matProp: THREE.MeshStandardMaterial;
  discMat: THREE.MeshBasicMaterial;
}

const R = ROTOR_RADIUS;
// vanes 1 & 3 share the "pitch/X" colour, vanes 2 & 4 the "roll/Y" colour
const VANE_COLOR = [COLOR.vanePitch, COLOR.vaneRoll, COLOR.vanePitch, COLOR.vaneRoll];

// The singlecopter, built in a local Z-up frame (matching Blender). The parent
// drives position / yaw / lean on the wrapping group, and prop spin + the four
// independent vane deflections (and highlight emissive) through the refs.
const Singlecopter = forwardRef<CopterRefs, { scale?: number }>(function Singlecopter(
  { scale = 1 },
  ref
) {
  const propG = useRef<THREE.Group>(null);
  const v1 = useRef<THREE.Group>(null);
  const v2 = useRef<THREE.Group>(null);
  const v3 = useRef<THREE.Group>(null);
  const v4 = useRef<THREE.Group>(null);

  const vaneMats = useMemo(
    () => VANE_COLOR.map((c) => new THREE.MeshStandardMaterial({
      color: c, metalness: 0.1, roughness: 0.5, emissive: c, emissiveIntensity: 0,
    })),
    []
  );
  const matProp = useMemo(
    () => new THREE.MeshStandardMaterial({ color: COLOR.prop, metalness: 0.3, roughness: 0.4, emissive: COLOR.prop, emissiveIntensity: 0 }),
    []
  );
  const discMat = useMemo(
    () => new THREE.MeshBasicMaterial({ color: COLOR.prop, transparent: true, opacity: 0, side: THREE.DoubleSide }),
    []
  );
  const matBody = useMemo(() => new THREE.MeshStandardMaterial({ color: COLOR.body, metalness: 0.4, roughness: 0.5 }), []);
  const matDark = useMemo(() => new THREE.MeshStandardMaterial({ color: "#0a0a0a", metalness: 0.6, roughness: 0.4 }), []);

  useImperativeHandle(ref, () => ({
    get prop() { return propG.current; },
    get vanes() { return [v1.current, v2.current, v3.current, v4.current]; },
    vaneMats, matProp, discMat,
  }), [vaneMats, matProp, discMat]);

  // X-arm fins (v1 fore +X, v3 aft −X): thin in X, hinge about Y
  const xFin = <boxGeometry args={[0.006, 0.1, 0.1]} />;
  // Y-arm fins (v2 left +Y, v4 right −Y): thin in Y, hinge about X
  const yFin = <boxGeometry args={[0.1, 0.006, 0.1]} />;

  return (
    <group scale={scale}>
      {/* central body / motor pod (cylinder along Z) */}
      <mesh castShadow rotation={[Math.PI / 2, 0, 0]} material={matBody}>
        <cylinderGeometry args={[0.05, 0.045, 0.22, 20]} />
      </mesh>
      <mesh position={[0, 0, -0.045]} castShadow material={matDark}>
        <boxGeometry args={[0.07, 0.05, 0.07]} />
      </mesh>
      <mesh position={[0, 0, 0.105]} rotation={[Math.PI / 2, 0, 0]} castShadow material={matDark}>
        <cylinderGeometry args={[0.03, 0.03, 0.04, 16]} />
      </mesh>

      {/* landing legs */}
      {[0, 1, 2, 3].map((i) => {
        const a = (i / 4) * Math.PI * 2 + Math.PI / 4;
        return (
          <mesh key={i} position={[Math.cos(a) * 0.06, Math.sin(a) * 0.06, -0.12]}
            rotation={[Math.cos(a) * 0.4, Math.sin(a) * 0.4, 0]} castShadow material={matDark}>
            <cylinderGeometry args={[0.004, 0.004, 0.12, 8]} />
          </mesh>
        );
      })}

      {/* propeller group (spins about Z) */}
      <group ref={propG} position={[0, 0, 0.135]}>
        <mesh material={matDark}>
          <cylinderGeometry args={[0.018, 0.018, 0.03, 12]} />
        </mesh>
        {[0, 1].map((b) => (
          <mesh key={b} rotation={[0, 0, b * Math.PI]} material={matProp}>
            <boxGeometry args={[2 * R, 0.028, 0.006]} />
          </mesh>
        ))}
        <mesh material={discMat}>
          <circleGeometry args={[R, 40]} />
        </mesh>
      </group>

      {/* four independent vanes, each hinged at the centre */}
      <group ref={v1} position={[0, 0, 0.02]}>
        <mesh position={[0.1, 0, 0]} castShadow material={vaneMats[0]}>{xFin}</mesh>
      </group>
      <group ref={v3} position={[0, 0, 0.02]}>
        <mesh position={[-0.1, 0, 0]} castShadow material={vaneMats[2]}>{xFin}</mesh>
      </group>
      <group ref={v2} position={[0, 0, 0.02]}>
        <mesh position={[0, 0.1, 0]} castShadow material={vaneMats[1]}>{yFin}</mesh>
      </group>
      <group ref={v4} position={[0, 0, 0.02]}>
        <mesh position={[0, -0.1, 0]} castShadow material={vaneMats[3]}>{yFin}</mesh>
      </group>
    </group>
  );
});

export default Singlecopter;
