import { forwardRef, useImperativeHandle, useMemo, useRef } from "react";
import * as THREE from "three";
import { COLOR, ROTOR_RADIUS } from "../consts";

// Handles the parent uses to animate the airframe each frame.
export interface CopterRefs {
  prop: THREE.Group | null;
  vanePitch: THREE.Group | null; // Vanes 1-3 (fore/aft)
  vaneRoll: THREE.Group | null; // Vanes 2-4 (left/right)
  matPitch: THREE.MeshStandardMaterial;
  matRoll: THREE.MeshStandardMaterial;
  matProp: THREE.MeshStandardMaterial;
  discMat: THREE.MeshBasicMaterial;
}

const R = ROTOR_RADIUS;

// The singlecopter, built in a local Z-up frame (matching Blender). The parent
// drives position / yaw / lean on the wrapping group, and prop spin + vane
// deflection (and highlight emissive) through the returned refs/materials.
const Singlecopter = forwardRef<CopterRefs, { scale?: number }>(function Singlecopter(
  { scale = 1 },
  ref
) {
  const propG = useRef<THREE.Group>(null);
  const pitchG = useRef<THREE.Group>(null);
  const rollG = useRef<THREE.Group>(null);

  // Material instances (stable) so the parent can pulse emissiveIntensity to
  // indicate which parts are actively moving.
  const matPitch = useMemo(
    () => new THREE.MeshStandardMaterial({ color: COLOR.vanePitch, metalness: 0.1, roughness: 0.5, emissive: COLOR.vanePitch, emissiveIntensity: 0 }),
    []
  );
  const matRoll = useMemo(
    () => new THREE.MeshStandardMaterial({ color: COLOR.vaneRoll, metalness: 0.1, roughness: 0.5, emissive: COLOR.vaneRoll, emissiveIntensity: 0 }),
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
    get vanePitch() { return pitchG.current; },
    get vaneRoll() { return rollG.current; },
    matPitch, matRoll, matProp, discMat,
  }), [matPitch, matRoll, matProp, discMat]);

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

      {/* Vanes 1-3 : pitch pair (fore +X / aft -X), hinge about Y */}
      <group ref={pitchG} position={[0, 0, 0.02]}>
        <mesh position={[0.1, 0, 0]} castShadow material={matPitch}>
          <boxGeometry args={[0.006, 0.1, 0.1]} />
        </mesh>
        <mesh position={[-0.1, 0, 0]} castShadow material={matPitch}>
          <boxGeometry args={[0.006, 0.1, 0.1]} />
        </mesh>
      </group>

      {/* Vanes 2-4 : roll pair (left +Y / right -Y), hinge about X */}
      <group ref={rollG} position={[0, 0, 0.02]}>
        <mesh position={[0, 0.1, 0]} castShadow material={matRoll}>
          <boxGeometry args={[0.1, 0.006, 0.1]} />
        </mesh>
        <mesh position={[0, -0.1, 0]} castShadow material={matRoll}>
          <boxGeometry args={[0.1, 0.006, 0.1]} />
        </mesh>
      </group>
    </group>
  );
});

export default Singlecopter;
</content>
