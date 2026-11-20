using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct VehicleChassisCollisionJob : IJob
    {
        public int ActiveVehicleCount;
        public float FixedDeltaTime;
        public float CapsuleRadius;     // ~0.95m (half vehicle width)
        public float CapsuleHalfSpanZ;  // ~1.15m (centers at z = +/-1.15m)
        public float Restitution;       // ~0.22f
        public float BodyFriction;      // ~0.25f

        public NativeArray<VehicleState> Vehicles;

        public void Execute()
        {
            float invDt = 1.0f / math.max(FixedDeltaTime, 1e-4f);
            float sumRadius = CapsuleRadius * 2.0f;
            float sumRadiusSq = sumRadius * sumRadius;

            for (int i = 0; i < ActiveVehicleCount; i++)
            {
                VehicleState vi = Vehicles[i];
                if ((vi.TierFlags & VehicleTierFlags.IsSleeping) != 0) continue;

                float3 fwdI = math.rotate(vi.Rotation, new float3(0f, 0f, CapsuleHalfSpanZ));
                float3 i0 = vi.Position + fwdI;
                float3 i1 = vi.Position - fwdI;

                for (int j = i + 1; j < ActiveVehicleCount; j++)
                {
                    VehicleState vj = Vehicles[j];
                    if ((vj.TierFlags & VehicleTierFlags.IsSleeping) != 0) continue;

                    // Quick coarse center-distance reject before 4-sphere capsule check
                    float coarseMax = sumRadius + CapsuleHalfSpanZ * 2.0f + 0.5f;
                    if (math.lengthsq(vi.Position - vj.Position) > coarseMax * coarseMax)
                        continue;

                    float3 fwdJ = math.rotate(vj.Rotation, new float3(0f, 0f, CapsuleHalfSpanZ));
                    float3 j0 = vj.Position + fwdJ;
                    float3 j1 = vj.Position - fwdJ;

                    float3 bestA = float3.zero, bestB = float3.zero;
                    float bestDistSq = float.MaxValue;

                    TestSpherePair(i0, j0, ref bestA, ref bestB, ref bestDistSq);
                    TestSpherePair(i0, j1, ref bestA, ref bestB, ref bestDistSq);
                    TestSpherePair(i1, j0, ref bestA, ref bestB, ref bestDistSq);
                    TestSpherePair(i1, j1, ref bestA, ref bestB, ref bestDistSq);

                    if (bestDistSq >= sumRadiusSq || bestDistSq < 1e-8f) continue;

                    float dist = math.sqrt(bestDistSq);
                    float penetration = sumRadius - dist;
                    float3 normal = (bestA - bestB) / dist; // Points from j -> i
                    float3 contactPt = 0.5f * (bestA + bestB);

                    float3 rI = contactPt - vi.Position;
                    float3 rJ = contactPt - vj.Position;

                    float3 vContactI = vi.LinearVelocity + math.cross(vi.AngularVelocity, rI);
                    float3 vContactJ = vj.LinearVelocity + math.cross(vj.AngularVelocity, rJ);
                    float3 vRel = vContactI - vContactJ;

                    float vN = math.dot(vRel, normal);
                    float biasVel = 0.20f * penetration * invDt;
                    if (vN > biasVel) continue; // Already separating fast enough

                    float kEff = ComputeEffectiveInvMass(in vi, in rI, in normal)
                               + ComputeEffectiveInvMass(in vj, in rJ, in normal);
                    float jN = math.max(0f, (-(1.0f + Restitution) * vN + biasVel) / math.max(kEff, 1e-5f));

                    // Tangential rubbing friction
                    float3 vTan = vRel - vN * normal;
                    float vTanMag = math.length(vTan);
                    float3 impulse = jN * normal;

                    if (vTanMag > 1e-4f)
                    {
                        float3 tanDir = vTan / vTanMag;
                        float kTan = ComputeEffectiveInvMass(in vi, in rI, in tanDir)
                                   + ComputeEffectiveInvMass(in vj, in rJ, in tanDir);
                        float jT = math.clamp(-vTanMag / math.max(kTan, 1e-5f), -BodyFriction * jN, BodyFriction * jN);
                        impulse += jT * tanDir;
                    }

                    vi.ExternalImpulse += impulse;
                    vi.ExternalAngularImpulse += math.cross(rI, impulse);
                    vi.TierFlags |= VehicleTierFlags.ExternalImpulsePending;

                    vj.ExternalImpulse -= impulse;
                    vj.ExternalAngularImpulse -= math.cross(rJ, impulse);
                    vj.TierFlags |= VehicleTierFlags.ExternalImpulsePending;
                    Vehicles[j] = vj;
                }

                Vehicles[i] = vi;
            }
        }

        private static void TestSpherePair(
            float3 a, float3 b, ref float3 bestA, ref float3 bestB, ref float bestDistSq)
        {
            float dSq = math.lengthsq(a - b);
            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                bestA = a;
                bestB = b;
            }
        }

        private static float ComputeEffectiveInvMass(in VehicleState v, in float3 r, in float3 dir)
        {
            float3 rCrossN = math.cross(r, dir);
            quaternion invRot = math.inverse(v.Rotation);
            float3 localInvI = v.InverseInertiaLocal;
            float3 angTerm = math.rotate(v.Rotation, localInvI * math.rotate(invRot, rCrossN));
            return v.InverseMass + math.dot(dir, math.cross(angTerm, r));
        }
    }
}
