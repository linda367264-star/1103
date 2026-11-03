using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace BoidsPlugin
{
    class Boid
{
    public Point3d Position;
    public Vector3d Velocity;
    public Vector3d Acceleration = Vector3d.Zero;
    public double MaxSpeed;
    public double MaxForce;

    public Boid(Point3d position, Vector3d velocity, double maxSpeed, double maxForce)
    {
        Position = position;
        Velocity = velocity;
        MaxSpeed = maxSpeed;
        MaxForce = maxForce;
    }

    public void Update(Point3d min, Point3d max)
    {
        Position = Position + Velocity;
        Velocity = Velocity + Acceleration;
        if (Velocity.Length > MaxSpeed)
            Velocity = Velocity * (MaxSpeed / Velocity.Length);
        Acceleration = Vector3d.Zero;
        Wrap(min, max);
    }

    // 穿牆：飛出盒子的一邊，就從對面那一邊進來（速度不變，不是反彈）
    void Wrap(Point3d min, Point3d max)
    {
        double x = Position.X, y = Position.Y, z = Position.Z;

        WrapAxis(ref x, min.X, max.X);
        WrapAxis(ref y, min.Y, max.Y);
        WrapAxis(ref z, min.Z, max.Z);

        Position = new Point3d(x, y, z);
    }

    // 只處理一個軸：position 超過 high 就移到 low 那一邊，小於 low 就移到 high 那一邊
    static void WrapAxis(ref double position, double low, double high)
    {
        if (position > high) position = low;
        else if (position < low) position = high;
    }

    // 規則 1：對齊　viewRadius 內鄰居的平均飛行方向 → 轉向力（沒有鄰居就是 0）
    public Vector3d Align(List<Boid> boids, double viewRadius)
    {
        Vector3d alignSum = Vector3d.Zero;
        int count = 0;

        foreach (Boid other in boids)
        {
            if (other == this) continue;
            if (Position.DistanceTo(other.Position) < viewRadius)
            {
                alignSum += other.Velocity;
                count++;
            }
        }

        if (count == 0) return Vector3d.Zero;
        return Steer(alignSum);
    }

    // TODO 規則 2：聚集　public Vector3d Cohesion(List<Boid> boids, double viewRadius)
    //                    鄰居平均位置 − 自己的位置 → Steer
    // TODO 規則 3：分離　public Vector3d Separation(List<Boid> boids, double separationRadius)
    //                    太近的鄰居：(自己位置 − 對方位置) / distance²（distance > 1e-6）加總 → Steer

    // 轉向力 ＝ 期望速度 − 目前速度，長度限制在 MaxForce
    public Vector3d Steer(Vector3d desired)
    {
        if (!desired.Unitize()) return Vector3d.Zero;
        Vector3d steering = desired * MaxSpeed - Velocity;
        if (steering.Length > MaxForce)
            steering = steering * (MaxForce / steering.Length);
        return steering;
    }
}
}