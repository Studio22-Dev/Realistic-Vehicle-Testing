using System;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;

namespace VehiclePhysics.Bridge
{
    public static class ColliderMetaFlags
    {
        public const byte None            = 0;
        public const byte IgnoreWheelRays = 1 << 0;
    }

    /// <summary>
    /// Unmanaged collider identity key supporting both legacy instance IDs
    /// and Unity 6.3+ / 6.6+ 32/64-bit EntityIds without truncation or deprecation warnings.
    /// </summary>
    public readonly struct ColliderIdKey : IEquatable<ColliderIdKey>
    {
#if UNITY_6000_3_OR_NEWER
        public readonly EntityId Value;
        public ColliderIdKey(EntityId value) => Value = value;
        public bool IsValid => Value != EntityId.None;
        public bool Equals(ColliderIdKey other) => Value == other.Value;
        public override int GetHashCode() => Value.GetHashCode();
        public static ColliderIdKey FromHit(in RaycastHit hit) => new ColliderIdKey(hit.colliderEntityId);
        public static ColliderIdKey FromObject(UnityEngine.Object obj) =>
            obj != null ? new ColliderIdKey(obj.GetEntityId()) : default;
#else
        public readonly int Value;
        public ColliderIdKey(int value) => Value = value;
        public bool IsValid => Value != 0;
        public bool Equals(ColliderIdKey other) => Value == other.Value;
        public override int GetHashCode() => Value;
        public static ColliderIdKey FromHit(in RaycastHit hit) => new ColliderIdKey(hit.colliderInstanceID);
        public static ColliderIdKey FromObject(UnityEngine.Object obj) =>
            obj != null ? new ColliderIdKey(obj.GetInstanceID()) : default;
#endif
        public override bool Equals(object obj) => obj is ColliderIdKey other && Equals(other);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ColliderPhysicsMeta // 16 bytes
    {
        public uint OwnerVehicleId;        // 0 = world/prop; >0 = skips rays for matching VehicleId
        public int DynamicBodyIndex;       // -1 = static; >=0 = index in DynamicBodies
        public byte SurfaceMaterialIndex;  // Index into VehicleSimulationWorld.SurfaceMaterials
        public byte Flags;                 // ColliderMetaFlags
        private ushort _pad0;
        private int _pad1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct DynamicBodyMotion // 48 bytes
    {
        public float3 WorldCenterOfMass;
        private float _pad0;
        public float3 LinearVelocity;
        private float _pad1;
        public float3 AngularVelocity;
        private float _pad2;

        public float3 GetPointVelocity(float3 worldPoint)
        {
            return LinearVelocity + math.cross(AngularVelocity, worldPoint - WorldCenterOfMass);
        }
    }
}
