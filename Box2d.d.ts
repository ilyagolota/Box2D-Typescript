module Box2D.Common {
    export class Color {
        constructor(rr: number, gg: number, bb: number);
        set(rr: number, gg: number, bb: number): void;
        r(rr: number): void;
        g(gg: number): void;
        b(bb: number): void;
        color: number;
    }

    export class Settings {
        static VERSION: string;
        static USHRT_MAX: number;
        static PI: number;
        static MAX_MANIFOLD_POINTS: number;
        static AABB_EXTENSION: number;
        static AABB_MULTIPLIER: number;
        static POLYGON_RADIUS: number;
        static LINEAR_SLOP: number;
        static ANGULAR_SLOP: number;
        static TOI_SLOP: number;
        static MAX_TOI_CONTACTS_PER_ISLAND: number;
        static MAX_TOI_JOINTS_PER_ISLAND: number;
        static VELOCITY_THRESHOLD: number;
        static MAX_LINEAR_CORRECTION: number;
        static MAX_ANGULAR_CORRECTION: number;
        static MAX_TRANSLATION: number;
        static MAX_TRANSLATION_SQUARED: number;
        static MAX_ROTATION: number;
        static MAX_ROTATION_SQUARED: number;
        static CONTACT_BAUMGARTE: number;
        static TIME_TO_SLEEP: number;
        static LINEAR_SLEEP_TOLERANCE: number;
        static ANGULAR_SLEEP_TOLERANCE: number;
        static mixFriction(friction1: number, friction2: number): number;
        static mixRestitution(restitution1: number, restitution2: number): number;
        static assert(a: bool): void;
    }
}
module Box2D.Common.Math {
    export class Mat22 {
        col1: Box2D.Common.Math.Vec2;
        col2: Box2D.Common.Math.Vec2;
        constructor();
        static fromAngle(angle: number): Box2D.Common.Math.Mat22;
        static fromVV(c1: Box2D.Common.Math.Vec2, c2: Box2D.Common.Math.Vec2): Box2D.Common.Math.Mat22;
        set(angle: number): void;
        setVV(c1: Box2D.Common.Math.Vec2, c2: Box2D.Common.Math.Vec2): void;
        copy(): Box2D.Common.Math.Mat22;
        setM(m: Box2D.Common.Math.Mat22): void;
        addM(m: Box2D.Common.Math.Mat22): void;
        setIdentity(): void;
        setZero(): void;
        getAngle(): number;
        getInverse(out: Box2D.Common.Math.Mat22): Box2D.Common.Math.Mat22;
        solve(out: Box2D.Common.Math.Vec2, bX: number, bY: number): Box2D.Common.Math.Vec2;
        abs(): void;
    }
    export class Mat33 {
        col1: Box2D.Common.Math.Vec3;
        col2: Box2D.Common.Math.Vec3;
        col3: Box2D.Common.Math.Vec3;
        constructor(c1?: Box2D.Common.Math.Vec3, c2?: Box2D.Common.Math.Vec3, c3?: Box2D.Common.Math.Vec3);
        setVVV(c1: Box2D.Common.Math.Vec3, c2: Box2D.Common.Math.Vec3, c3: Box2D.Common.Math.Vec3): void;
        copy(): Box2D.Common.Math.Mat33;
        setM(m: Box2D.Common.Math.Mat33): void;
        addM(m: Box2D.Common.Math.Mat33): void;
        setIdentity(): void;
        setZero(): void;
        solve22(out: Box2D.Common.Math.Vec2, bX: number, bY: number): Box2D.Common.Math.Vec2;
        solve33(out: Box2D.Common.Math.Vec3, bX: number, bY: number, bZ: number): Box2D.Common.Math.Vec3;
    }
    export class Math {
        static VEC2_ZERO: Box2D.Common.Math.Vec2;
        static MAT22_IDENTITY: Box2D.Common.Math.Mat22;
        static TRANSFORM_IDENTITY: Box2D.Common.Math.Transform;
        static isValid(x: number): bool;
        static dot(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): number;
        static crossVV(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): number;
        static crossVF(a: Box2D.Common.Math.Vec2, s: number): Box2D.Common.Math.Vec2;
        static crossFV(s: number, a: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static mulMV(A: Box2D.Common.Math.Mat22, v: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static mulTMV(A: Box2D.Common.Math.Mat22, v: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static mulX(T: Box2D.Common.Math.Transform, v: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static mulXT(T: Box2D.Common.Math.Transform, v: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static addVV(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static subtractVV(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static distance(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): number;
        static distanceSquared(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): number;
        static mulFV(s: number, a: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static addMM(A: Box2D.Common.Math.Mat22, B: Box2D.Common.Math.Mat22): Box2D.Common.Math.Mat22;
        static mulMM(A: Box2D.Common.Math.Mat22, B: Box2D.Common.Math.Mat22): Box2D.Common.Math.Mat22;
        static mulTMM(A: Box2D.Common.Math.Mat22, B: Box2D.Common.Math.Mat22): Box2D.Common.Math.Mat22;
        static abs(a: number): number;
        static absV(a: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static absM(A: Box2D.Common.Math.Mat22): Box2D.Common.Math.Mat22;
        static min(a: number, b: number): number;
        static minV(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static max(a: number, b: number): number;
        static maxV(a: Box2D.Common.Math.Vec2, b: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static clamp(a: number, low: number, high: number): number;
        static clampV(a: Box2D.Common.Math.Vec2, low: Box2D.Common.Math.Vec2, high: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        static swap(a: any[], b: any[]): void;
        static random(): number;
        static randomRange(lo: number, hi: number): number;
        static nextPowerOfTwo(x: number): number;
        static isPowerOfTwo(x: number): bool;
    }
    export class Sweep {
        localCenter: Box2D.Common.Math.Vec2;
        c0: Box2D.Common.Math.Vec2;
        c: Box2D.Common.Math.Vec2;
        a0: number;
        a: number;
        t0: number;
        set(other: Box2D.Common.Math.Sweep): void;
        copy(): Box2D.Common.Math.Sweep;
        getTransform(xf: Box2D.Common.Math.Transform, alpha: number): void;
        advance(t: number): void;
    }
    export class Transform {
        position: Box2D.Common.Math.Vec2;
        R: Box2D.Common.Math.Mat22;
        constructor(pos?: Box2D.Common.Math.Vec2, r?: Box2D.Common.Math.Mat22);
        initialize(pos: Box2D.Common.Math.Vec2, r: Box2D.Common.Math.Mat22): void;
        setIdentity(): void;
        set(x: Box2D.Common.Math.Transform): void;
        getAngle(): number;
    }
    export class Vec2 {
        x: number;
        y: number;
        constructor(x_?: number, y_?: number);
        setZero(): void;
        set(x_?: number, y_?: number): void;
        setV(v: Box2D.Common.Math.Vec2): void;
        getNegative(): Box2D.Common.Math.Vec2;
        negativeSelf(): void;
        static make(x_: number, y_: number): Box2D.Common.Math.Vec2;
        copy(): Box2D.Common.Math.Vec2;
        add(v: Box2D.Common.Math.Vec2): void;
        subtract(v: Box2D.Common.Math.Vec2): void;
        multiply(a: number): void;
        mulM(A: Box2D.Common.Math.Mat22): void;
        mulTM(A: Box2D.Common.Math.Mat22): void;
        crossVF(s: number): void;
        crossFV(s: number): void;
        minV(b: Box2D.Common.Math.Vec2): void;
        maxV(b: Box2D.Common.Math.Vec2): void;
        abs(): void;
        length(): number;
        lengthSquared(): number;
        normalize(): number;
        isValid(): bool;
    }
    export class Vec3 {
        x: number;
        y: number;
        z: number;
        constructor(x?: number, y?: number, z?: number);
        setZero(): void;
        set(x: number, y: number, z: number): void;
        setV(v: Box2D.Common.Math.Vec3): void;
        getNegative(): Box2D.Common.Math.Vec3;
        negativeSelf(): void;
        copy(): Box2D.Common.Math.Vec3;
        add(v: Box2D.Common.Math.Vec3): void;
        subtract(v: Box2D.Common.Math.Vec3): void;
        multiply(a: number): void;
    }
}
module Box2D.Collision {
    export class RayCastInput {
        p1: Box2D.Common.Math.Vec2;
        p2: Box2D.Common.Math.Vec2;
        maxFraction: number;
    }
    export class RayCastOutput {
        normal: Box2D.Common.Math.Vec2;
        fraction: number;
    }
    export class AABB {
        lowerBound: Box2D.Common.Math.Vec2;
        upperBound: Box2D.Common.Math.Vec2;
        isValid(): bool;
        getCenter(): Box2D.Common.Math.Vec2;
        getExtents(): Box2D.Common.Math.Vec2;
        contains(aabb: Box2D.Collision.AABB): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput): bool;
        testOverlap(other: Box2D.Collision.AABB): bool;
        static combine(aabb1: Box2D.Collision.AABB, aab: Box2D.Collision.AABB): Box2D.Collision.AABB;
        combine(aabb1: Box2D.Collision.AABB, aab: Box2D.Collision.AABB): void;
    }
    export class Proxy {
        lowerBounds: number[];
        upperBounds: number[];
        overlapCount: number;
        timeStamp: number;
        pairs: {[s: string]: any;};
        next: Box2D.Collision.Proxy;
        userData: any;
        isValid(): bool;
    }
    export class Bound {
        value: number;
        proxy: Box2D.Collision.Proxy;
        stabbingCount: number;
        isLower(): bool;
        isUpper(): bool;
        swap(b: Box2D.Collision.Bound): void;
    }
    export class BoundValues {
        lowerValues: number[];
        upperValues: number[];
        constructor();
    }
    export interface IBroadPhase {
    }
    export class BroadPhase implements IBroadPhase {
        static S_VALIDATE: bool;
        static INVALID: number;
        static NULL_EDGE: number;
        constructor(worldAABB: Box2D.Collision.AABB);
        inRange(aabb: Box2D.Collision.AABB): bool;
        createProxy(aabb: Box2D.Collision.AABB, userData: any): any;
        destroyProxy(proxy_: any): void;
        moveProxy(proxy_: any, aabb: Box2D.Collision.AABB, displacement: Box2D.Common.Math.Vec2): void;
        updatePairs(callback: (data1: any, data2: any) => void): void;
        testOverlap(proxyA: any, proxyB: any): bool;
        getUserData(proxy: any): any;
        getFatAABB(proxy_: any): Box2D.Collision.AABB;
        getProxyCount(): number;
        query(callback: () => any, aabb: Box2D.Collision.AABB): void;
        validate(): void;
        rebalance(iterations: number): void;
        rayCast(callback: () => any, input: Box2D.Collision.RayCastInput): void;
        testOverlapBound(b: Box2D.Collision.BoundValues, p: Box2D.Collision.Proxy): bool;
        static binarySearch(bounds: Box2D.Collision.Bound[], count: number, value: number): number;
    }
    export class Features {
        referenceEdge: number;
        incidentEdge: number;
        incidentVertex: number;
        flip: number;
    }
    export class ContactID {
        features: Box2D.Collision.Features;
        constructor();
        set(id: Box2D.Collision.ContactID): void;
        copy(): Box2D.Collision.ContactID;
        key: number;
    }
    export class ClipVertex {
        v: Box2D.Common.Math.Vec2;
        id: Box2D.Collision.ContactID;
        set(other: Box2D.Collision.ClipVertex): void;
    }
    export class Collision {
        static NULL_FEATURE: number;
        static clipSegmentToLine(vOut: Box2D.Collision.ClipVertex[], vIn: Box2D.Collision.ClipVertex[], normal: Box2D.Common.Math.Vec2, offset: number): number;
        static edgeSeparation(poly1: Box2D.Collision.Shapes.PolygonShape, xf1: Box2D.Common.Math.Transform, edge1: number, poly2: Box2D.Collision.Shapes.PolygonShape, xf2: Box2D.Common.Math.Transform): number;
        static findMaxSeparation(edgeIndex: number[], poly1: Box2D.Collision.Shapes.PolygonShape, xf1: Box2D.Common.Math.Transform, poly2: Box2D.Collision.Shapes.PolygonShape, xf2: Box2D.Common.Math.Transform): number;
        static findIncidentEdge(c: Box2D.Collision.ClipVertex[], poly1: Box2D.Collision.Shapes.PolygonShape, xf1: Box2D.Common.Math.Transform, edge1: number, poly2: Box2D.Collision.Shapes.PolygonShape, xf2: Box2D.Common.Math.Transform): void;
        static collidePolygons(manifold: Box2D.Collision.Manifold, polyA: Box2D.Collision.Shapes.PolygonShape, xfA: Box2D.Common.Math.Transform, polyB: Box2D.Collision.Shapes.PolygonShape, xfB: Box2D.Common.Math.Transform): void;
        static collideCircles(manifold: Box2D.Collision.Manifold, circle1: Box2D.Collision.Shapes.CircleShape, xf1: Box2D.Common.Math.Transform, circle2: Box2D.Collision.Shapes.CircleShape, xf2: Box2D.Common.Math.Transform): void;
        static collidePolygonAndCircle(manifold: Box2D.Collision.Manifold, polygon: Box2D.Collision.Shapes.PolygonShape, xf1: Box2D.Common.Math.Transform, circle: Box2D.Collision.Shapes.CircleShape, xf2: Box2D.Common.Math.Transform): void;
        static testOverlap(a: Box2D.Collision.AABB, b: Box2D.Collision.AABB): bool;
    }
    export class ContactPoint {
        shape1: Box2D.Collision.Shapes.Shape;
        shape2: Box2D.Collision.Shapes.Shape;
        position: Box2D.Common.Math.Vec2;
        velocity: Box2D.Common.Math.Vec2;
        normal: Box2D.Common.Math.Vec2;
        separation: number;
        friction: number;
        restitution: number;
        id: Box2D.Collision.ContactID;
    }
    export class Distance {
        static distance(output: Box2D.Collision.DistanceOutput, cache: Box2D.Collision.SimplexCache, input: Box2D.Collision.DistanceInput): void;
    }
    export class DistanceInput {
        proxyA: Box2D.Collision.DistanceProxy;
        proxyB: Box2D.Collision.DistanceProxy;
        transformA: Box2D.Common.Math.Transform;
        transformB: Box2D.Common.Math.Transform;
        useRadii: bool;
    }
    export class DistanceOutput {
        pointA: Box2D.Common.Math.Vec2;
        pointB: Box2D.Common.Math.Vec2;
        distance: number;
        iterations: number;
    }
    export class DistanceProxy {
        m_vertices: Box2D.Common.Math.Vec2[];
        m_count: number;
        m_radius: number;
        set(shape: Box2D.Collision.Shapes.Shape): void;
        getSupport(d: Box2D.Common.Math.Vec2): number;
        getSupportVertex(d: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getVertexCount(): number;
        getVertex(index: number): Box2D.Common.Math.Vec2;
    }
    export class DynamicTree {
        constructor();
        createProxy(aabb: Box2D.Collision.AABB, userData: any): Box2D.Collision.DynamicTreeNode;
        destroyProxy(proxy: Box2D.Collision.DynamicTreeNode): void;
        moveProxy(proxy: Box2D.Collision.DynamicTreeNode, aabb: Box2D.Collision.AABB, displacement: Box2D.Common.Math.Vec2): bool;
        rebalance(iterations: number): void;
        getFatAABB(proxy: Box2D.Collision.DynamicTreeNode): Box2D.Collision.AABB;
        getUserData(proxy: Box2D.Collision.DynamicTreeNode): any;
        query(callback: (n: DynamicTreeNode) => bool, aabb: Box2D.Collision.AABB): void;
        rayCast(callback: (i: RayCastInput, n: DynamicTreeNode) => number, input: Box2D.Collision.RayCastInput): void;
    }
    export class DynamicTreeBroadPhase implements IBroadPhase {
        createProxy(aabb: Box2D.Collision.AABB, userData: any): any;
        destroyProxy(proxy: any): void;
        moveProxy(proxy: any, aabb: Box2D.Collision.AABB, displacement: Box2D.Common.Math.Vec2): void;
        testOverlap(proxyA: any, proxyB: any): bool;
        getUserData(proxy: any): any;
        getFatAABB(proxy: any): Box2D.Collision.AABB;
        getProxyCount(): number;
        updatePairs(callback: (d1: any, d2: any) => void): void;
        query(callback: (n: DynamicTreeNode) => bool, aabb: Box2D.Collision.AABB): void;
        rayCast(callback: (i: RayCastInput, n: DynamicTreeNode) => number, input: Box2D.Collision.RayCastInput): void;
        validate(): void;
        rebalance(iterations: number): void;
    }
    export class DynamicTreeNode {
        userData: any;
        aabb: Box2D.Collision.AABB;
        parent: Box2D.Collision.DynamicTreeNode;
        child1: Box2D.Collision.DynamicTreeNode;
        child2: Box2D.Collision.DynamicTreeNode;
        isLeaf(): bool;
    }
    export class DynamicTreePair {
        proxyA: Box2D.Collision.DynamicTreeNode;
        proxyB: Box2D.Collision.DynamicTreeNode;
    }
    export class Manifold {
        m_points: Box2D.Collision.ManifoldPoint[];
        m_localPlaneNormal: Box2D.Common.Math.Vec2;
        m_localPoint: Box2D.Common.Math.Vec2;
        m_type: number;
        m_pointCount: number;
        static CIRCLES: number;
        static FACE_A: number;
        static FACE_B: number;
        constructor();
        reset(): void;
        set(m: Box2D.Collision.Manifold): void;
        copy(): Box2D.Collision.Manifold;
    }
    export class ManifoldPoint {
        m_localPoint: Box2D.Common.Math.Vec2;
        m_normalImpulse: number;
        m_tangentImpulse: number;
        m_id: Box2D.Collision.ContactID;
        constructor();
        reset(): void;
        set(m: Box2D.Collision.ManifoldPoint): void;
    }
    export class OBB {
        R: Box2D.Common.Math.Mat22;
        center: Box2D.Common.Math.Vec2;
        extents: Box2D.Common.Math.Vec2;
    }
    export class Pair {
        userData: any;
        proxy1: Box2D.Collision.Proxy;
        proxy2: Box2D.Collision.Proxy;
        next: Box2D.Collision.Pair;
        status: number;
        static NULL_PROXY: number;
        static PAIR_BUFFERED: number;
        static PAIR_REMOVED: number;
        static PAIR_FINAL: number;
        setBuffered(): void;
        clearBuffered(): void;
        isBuffered(): bool;
        setRemoved(): void;
        clearRemoved(): void;
        isRemoved(): bool;
        setFinal(): void;
        isFinal(): bool;
    }
    export class PairManager {
        constructor();
        initialize(broadPhase: Box2D.Collision.BroadPhase): void;
        addBufferedPair(proxy1: Box2D.Collision.Proxy, proxy2: Box2D.Collision.Proxy): void;
        removeBufferedPair(proxy1: Box2D.Collision.Proxy, proxy2: Box2D.Collision.Proxy): void;
        commit(callback: (data1: any, data2: any) => void): void;
    }
    export class Point {
        p: Box2D.Common.Math.Vec2;
        support(xf: Box2D.Common.Math.Transform, vX: number, vY: number): Box2D.Common.Math.Vec2;
        getFirstVertex(xf: Box2D.Common.Math.Transform): Box2D.Common.Math.Vec2;
    }
    export class Segment {
        p1: Box2D.Common.Math.Vec2;
        p2: Box2D.Common.Math.Vec2;
        testSegment(lambda: any[], normal: Box2D.Common.Math.Vec2, segment: Box2D.Collision.Segment, maxLambda: number): bool;
        extend(aabb: Box2D.Collision.AABB): void;
        extendForward(aabb: Box2D.Collision.AABB): void;
        extendBackward(aabb: Box2D.Collision.AABB): void;
    }
    export class SimplexCache {
        metric: number;
        count: number;
        indexA: number[];
        indexB: number[];
    }
    export class TimeOfImpact {
        static timeOfImpact(input: Box2D.Collision.TOIInput): number;
    }
    export class TOIInput {
        proxyA: Box2D.Collision.DistanceProxy;
        proxyB: Box2D.Collision.DistanceProxy;
        sweepA: Box2D.Common.Math.Sweep;
        sweepB: Box2D.Common.Math.Sweep;
        tolerance: number;
    }
    export class WorldManifold {
        m_normal: Box2D.Common.Math.Vec2;
        m_points: Box2D.Common.Math.Vec2[];
        constructor();
        initialize(manifold: Box2D.Collision.Manifold, xfA: Box2D.Common.Math.Transform, radiusA: number, xfB: Box2D.Common.Math.Transform, radiusB: number): void;
    }
}
module Box2D.Collision.Shapes {
    export class MassData {
        mass: number;
        center: Box2D.Common.Math.Vec2;
        I: number;
    }
    export class EdgeChainDef {
        vertices: any[];
        vertexCount: number;
        isALoop: bool;
        constructor();
    }
    export class Shape {
        static HIT_COLLIDE: number;
        static MISS_COLLIDE: number;
        static STARTS_INSIDE_COLLIDE: number;
        copy(): Box2D.Collision.Shapes.Shape;
        set(other: Box2D.Collision.Shapes.Shape): void;
        getType(): number;
        testPoint(xf: Box2D.Common.Math.Transform, p: Box2D.Common.Math.Vec2): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput, transform: Box2D.Common.Math.Transform): bool;
        computeAABB(aabb: Box2D.Collision.AABB, xf: Box2D.Common.Math.Transform): void;
        computeMass(massData: Box2D.Collision.Shapes.MassData, density: number): void;
        computeSubmergedArea(normal: Box2D.Common.Math.Vec2, offset: number, xf: Box2D.Common.Math.Transform, c: Box2D.Common.Math.Vec2): number;
        static testOverlap(shape1: Box2D.Collision.Shapes.Shape, transform1: Box2D.Common.Math.Transform, shape2: Box2D.Collision.Shapes.Shape, transform2: Box2D.Common.Math.Transform): bool;
        constructor();
    }
    export class CircleShape extends Shape {
        copy(): Box2D.Collision.Shapes.Shape;
        set(other: Box2D.Collision.Shapes.Shape): void;
        testPoint(transform: Box2D.Common.Math.Transform, p: Box2D.Common.Math.Vec2): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput, transform: Box2D.Common.Math.Transform): bool;
        computeAABB(aabb: Box2D.Collision.AABB, transform: Box2D.Common.Math.Transform): void;
        computeMass(massData: Box2D.Collision.Shapes.MassData, density: number): void;
        computeSubmergedArea(normal: Box2D.Common.Math.Vec2, offset: number, xf: Box2D.Common.Math.Transform, c: Box2D.Common.Math.Vec2): number;
        getLocalPosition(): Box2D.Common.Math.Vec2;
        setLocalPosition(position: Box2D.Common.Math.Vec2): void;
        getRadius(): number;
        setRadius(radius: number): void;
        constructor(radius?: number);
    }
    export class EdgeShape extends Shape {
        testPoint(transform: Box2D.Common.Math.Transform, p: Box2D.Common.Math.Vec2): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput, transform: Box2D.Common.Math.Transform): bool;
        computeAABB(aabb: Box2D.Collision.AABB, transform: Box2D.Common.Math.Transform): void;
        computeMass(massData: Box2D.Collision.Shapes.MassData, density: number): void;
        computeSubmergedArea(normal: Box2D.Common.Math.Vec2, offset: number, xf: Box2D.Common.Math.Transform, c: Box2D.Common.Math.Vec2): number;
        getLength(): number;
        getVertex1(): Box2D.Common.Math.Vec2;
        getVertex2(): Box2D.Common.Math.Vec2;
        getCoreVertex1(): Box2D.Common.Math.Vec2;
        getCoreVertex2(): Box2D.Common.Math.Vec2;
        getNormalVector(): Box2D.Common.Math.Vec2;
        getDirectionVector(): Box2D.Common.Math.Vec2;
        getCorner1Vector(): Box2D.Common.Math.Vec2;
        getCorner2Vector(): Box2D.Common.Math.Vec2;
        corner1IsConvex(): bool;
        corner2IsConvex(): bool;
        getFirstVertex(xf: Box2D.Common.Math.Transform): Box2D.Common.Math.Vec2;
        getNextEdge(): Box2D.Collision.Shapes.EdgeShape;
        getPrevEdge(): Box2D.Collision.Shapes.EdgeShape;
        support(xf: Box2D.Common.Math.Transform, dX: number, dY: number): Box2D.Common.Math.Vec2;
        constructor(v1: Box2D.Common.Math.Vec2, v2: Box2D.Common.Math.Vec2);
    }
    export class PolygonShape extends Shape {
        copy(): Box2D.Collision.Shapes.Shape;
        set(other: Box2D.Collision.Shapes.Shape): void;
        setAsArray(vertices: any[], vertexCount?: number): void;
        static asArray(vertices: any[], vertexCount: number): Box2D.Collision.Shapes.PolygonShape;
        setAsVector(vertices: Box2D.Common.Math.Vec2[], vertexCount?: number): void;
        static asVector(vertices: Box2D.Common.Math.Vec2[], vertexCount: number): Box2D.Collision.Shapes.PolygonShape;
        setAsBox(hx: number, hy: number): void;
        static asBox(hx: number, hy: number): Box2D.Collision.Shapes.PolygonShape;
        setAsOrientedBox(hx: number, hy: number, center?: Box2D.Common.Math.Vec2, angle?: number): void;
        static asOrientedBox(hx: number, hy: number, center?: Box2D.Common.Math.Vec2, angle?: number): Box2D.Collision.Shapes.PolygonShape;
        setAsEdge(v1: Box2D.Common.Math.Vec2, v2: Box2D.Common.Math.Vec2): void;
        static asEdge(v1: Box2D.Common.Math.Vec2, v2: Box2D.Common.Math.Vec2): Box2D.Collision.Shapes.PolygonShape;
        testPoint(xf: Box2D.Common.Math.Transform, p: Box2D.Common.Math.Vec2): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput, transform: Box2D.Common.Math.Transform): bool;
        computeAABB(aabb: Box2D.Collision.AABB, xf: Box2D.Common.Math.Transform): void;
        computeMass(massData: Box2D.Collision.Shapes.MassData, density: number): void;
        computeSubmergedArea(normal: Box2D.Common.Math.Vec2, offset: number, xf: Box2D.Common.Math.Transform, c: Box2D.Common.Math.Vec2): number;
        getVertexCount(): number;
        getVertices(): Box2D.Common.Math.Vec2[];
        getNormals(): Box2D.Common.Math.Vec2[];
        getSupport(d: Box2D.Common.Math.Vec2): number;
        getSupportVertex(d: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        constructor();
        static computeCentroid(vs: Box2D.Common.Math.Vec2[], count: number): Box2D.Common.Math.Vec2;
    }
}
module Box2D.Dynamics {
    export class Body {
        static STATIC_BODY: number;
        static KINEMATIC_BODY: number;
        static DYNAMIC_BODY: number;
        createFixture(def: Box2D.Dynamics.FixtureDef): Box2D.Dynamics.Fixture;
        createFixture2(shape: Box2D.Collision.Shapes.Shape, density?: number): Box2D.Dynamics.Fixture;
        destroyFixture(fixture: Box2D.Dynamics.Fixture): void;
        setPositionAndAngle(position: Box2D.Common.Math.Vec2, angle: number): void;
        setTransform(xf: Box2D.Common.Math.Transform): void;
        getTransform(): Box2D.Common.Math.Transform;
        getPosition(): Box2D.Common.Math.Vec2;
        setPosition(position: Box2D.Common.Math.Vec2): void;
        getAngle(): number;
        setAngle(angle: number): void;
        getWorldCenter(): Box2D.Common.Math.Vec2;
        getLocalCenter(): Box2D.Common.Math.Vec2;
        setLinearVelocity(v: Box2D.Common.Math.Vec2): void;
        getLinearVelocity(): Box2D.Common.Math.Vec2;
        setAngularVelocity(omega: number): void;
        getAngularVelocity(): number;
        getDefinition(): Box2D.Dynamics.BodyDef;
        applyForce(force: Box2D.Common.Math.Vec2, point: Box2D.Common.Math.Vec2): void;
        applyTorque(torque: number): void;
        applyImpulse(impulse: Box2D.Common.Math.Vec2, point: Box2D.Common.Math.Vec2): void;
        split(callback: (f: Fixture) => bool): Box2D.Dynamics.Body;
        merge(other: Box2D.Dynamics.Body): void;
        getMass(): number;
        getInertia(): number;
        getMassData(data: Box2D.Collision.Shapes.MassData): void;
        setMassData(massData: Box2D.Collision.Shapes.MassData): void;
        resetMassData(): void;
        getWorldPoint(localPoint: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getWorldVector(localVector: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getLocalPoint(worldPoint: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getLocalVector(worldVector: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getLinearVelocityFromWorldPoint(worldPoint: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getLinearVelocityFromLocalPoint(localPoint: Box2D.Common.Math.Vec2): Box2D.Common.Math.Vec2;
        getLinearDamping(): number;
        setLinearDamping(linearDamping: number): void;
        getAngularDamping(): number;
        setAngularDamping(angularDamping: number): void;
        setType(type: number): void;
        getType(): number;
        setBullet(flag: bool): void;
        isBullet(): bool;
        setSleepingAllowed(flag: bool): void;
        setAwake(flag: bool): void;
        isAwake(): bool;
        setFixedRotation(fixed: bool): void;
        isFixedRotation(): bool;
        setActive(flag: bool): void;
        isActive(): bool;
        isSleepingAllowed(): bool;
        getFixtureList(): Box2D.Dynamics.Fixture;
        getJointList(): Box2D.Dynamics.Joints.JointEdge;
        getControllerList(): Box2D.Dynamics.Controllers.ControllerEdge;
        getContactList(): Box2D.Dynamics.Contacts.ContactEdge;
        getNext(): Box2D.Dynamics.Body;
        getUserData(): any;
        setUserData(data: any): void;
        getWorld(): Box2D.Dynamics.World;
        constructor(bd: Box2D.Dynamics.BodyDef, world: Box2D.Dynamics.World);
    }
}
module Box2D.Dynamics {
    export class BodyDef {
        type: number;
        position: Box2D.Common.Math.Vec2;
        angle: number;
        linearVelocity: Box2D.Common.Math.Vec2;
        angularVelocity: number;
        linearDamping: number;
        angularDamping: number;
        allowSleep: bool;
        awake: bool;
        fixedRotation: bool;
        bullet: bool;
        active: bool;
        userData: any;
        inertiaScale: number;
        constructor();
    }
}
module Box2D.Dynamics {
    export class ContactFilter {
        shouldCollide(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): bool;
        rayCollide(userData: any, fixture: Box2D.Dynamics.Fixture): bool;
    }
}
module Box2D.Dynamics {
    export class ContactImpulse {
        normalImpulses: number[];
        tangentImpulses: number[];
    }
}
module Box2D.Dynamics {
    export class ContactListener {
        beginContact(contact: Box2D.Dynamics.Contacts.Contact): void;
        endContact(contact: Box2D.Dynamics.Contacts.Contact): void;
        preSolve(contact: Box2D.Dynamics.Contacts.Contact, oldManifold: Box2D.Collision.Manifold): void;
        postSolve(contact: Box2D.Dynamics.Contacts.Contact, impulse: Box2D.Dynamics.ContactImpulse): void;
    }
}
module Box2D.Dynamics {
    export class ContactManager {
        constructor();
        addPair(proxyUserDataA: any, proxyUserDataB: any): void;
        findNewContacts(): void;
        destroy(c: Box2D.Dynamics.Contacts.Contact): void;
        collide(): void;
    }
}
module Box2D.Dynamics {
    export class DebugDraw {
        static SHAPE_BIT: number;
        static JOINT_BIT: number;
        static AABB_BIT: number;
        static PAIR_BIT: number;
        static CENTER_OF_MASS_BIT: number;
        static CONTROLLER_BIT: number;
        constructor();
        setFlags(flags: number): void;
        getFlags(): number;
        appendFlags(flags: number): void;
        clearFlags(flags: number): void;
        setSprite(sprite: any): void;
        getSprite(): any;
        setDrawScale(drawScale: number): void;
        getDrawScale(): number;
        setLineThickness(lineThickness: number): void;
        getLineThickness(): number;
        setAlpha(alpha: number): void;
        getAlpha(): number;
        setFillAlpha(alpha: number): void;
        getFillAlpha(): number;
        setXFormScale(xformScale: number): void;
        getXFormScale(): number;
        drawPolygon(vertices: any[], vertexCount: number, color: Box2D.Common.Color): void;
        drawSolidPolygon(vertices: Box2D.Common.Math.Vec2[], vertexCount: number, color: Box2D.Common.Color): void;
        drawCircle(center: Box2D.Common.Math.Vec2, radius: number, color: Box2D.Common.Color): void;
        drawSolidCircle(center: Box2D.Common.Math.Vec2, radius: number, axis: Box2D.Common.Math.Vec2, color: Box2D.Common.Color): void;
        drawSegment(p1: Box2D.Common.Math.Vec2, p2: Box2D.Common.Math.Vec2, color: Box2D.Common.Color): void;
        drawTransform(xf: Box2D.Common.Math.Transform): void;
    }
}
module Box2D.Dynamics {
    export class DestructionListener {
        sayGoodbyeJoint(joint: Box2D.Dynamics.Joints.Joint): void;
        sayGoodbyeFixture(fixture: Box2D.Dynamics.Fixture): void;
    }
}
module Box2D.Dynamics {
    export class FilterData {
        categoryBits: number;
        maskBits: number;
        groupIndex: number;
        copy(): Box2D.Dynamics.FilterData;
    }
}
module Box2D.Dynamics {
    export class Fixture {
        getType(): number;
        getShape(): Box2D.Collision.Shapes.Shape;
        setSensor(sensor: bool): void;
        isSensor(): bool;
        setFilterData(filter: Box2D.Dynamics.FilterData): void;
        getFilterData(): Box2D.Dynamics.FilterData;
        getBody(): Box2D.Dynamics.Body;
        getNext(): Box2D.Dynamics.Fixture;
        getUserData(): any;
        setUserData(data: any): void;
        testPoint(p: Box2D.Common.Math.Vec2): bool;
        rayCast(output: Box2D.Collision.RayCastOutput, input: Box2D.Collision.RayCastInput): bool;
        getMassData(massData?: Box2D.Collision.Shapes.MassData): Box2D.Collision.Shapes.MassData;
        setDensity(density: number): void;
        getDensity(): number;
        getFriction(): number;
        setFriction(friction: number): void;
        getRestitution(): number;
        setRestitution(restitution: number): void;
        getAABB(): Box2D.Collision.AABB;
        constructor();
    }
}
module Box2D.Dynamics {
    export class FixtureDef {
        shape: Box2D.Collision.Shapes.Shape;
        userData: any;
        friction: number;
        restitution: number;
        density: number;
        isSensor: bool;
        filter: Box2D.Dynamics.FilterData;
        constructor();
    }
}
module Box2D.Dynamics {
    export class Island {
        constructor();
        initialize(bodyCapacity: number, contactCapacity: number, jointCapacity: number, allocator: any, listener: Box2D.Dynamics.ContactListener, contactSolver: Box2D.Dynamics.Contacts.ContactSolver): void;
        clear(): void;
        solve(step: Box2D.Dynamics.TimeStep, gravity: Box2D.Common.Math.Vec2, allowSleep: bool): void;
        solveTOI(subStep: Box2D.Dynamics.TimeStep): void;
        report(constraints: Box2D.Dynamics.Contacts.ContactConstraint[]): void;
        addBody(body: Box2D.Dynamics.Body): void;
        addContact(contact: Box2D.Dynamics.Contacts.Contact): void;
        addJoint(joint: Box2D.Dynamics.Joints.Joint): void;
    }
}
module Box2D.Dynamics {
    export class TimeStep {
        dt: number;
        inv_dt: number;
        dtRatio: number;
        velocityIterations: number;
        positionIterations: number;
        warmStarting: bool;
        set(step: Box2D.Dynamics.TimeStep): void;
    }
}
module Box2D.Dynamics {
    export class World {
        static NEW_FIXTURE: number;
        static LOCKED: number;
        constructor(gravity: Box2D.Common.Math.Vec2, doSleep: bool);
        setDestructionListener(listener: Box2D.Dynamics.DestructionListener): void;
        setContactFilter(filter: Box2D.Dynamics.ContactFilter): void;
        setContactListener(listener: Box2D.Dynamics.ContactListener): void;
        setDebugDraw(debugDraw: Box2D.Dynamics.DebugDraw): void;
        setBroadPhase(broadPhase: Box2D.Collision.IBroadPhase): void;
        validate(): void;
        getProxyCount(): number;
        createBody(def: Box2D.Dynamics.BodyDef): Box2D.Dynamics.Body;
        destroyBody(b: Box2D.Dynamics.Body): void;
        createJoint(def: Box2D.Dynamics.Joints.JointDef): Box2D.Dynamics.Joints.Joint;
        destroyJoint(j: Box2D.Dynamics.Joints.Joint): void;
        addController(c: Box2D.Dynamics.Controllers.Controller): Box2D.Dynamics.Controllers.Controller;
        removeController(c: Box2D.Dynamics.Controllers.Controller): void;
        createController(controller: Box2D.Dynamics.Controllers.Controller): Box2D.Dynamics.Controllers.Controller;
        destroyController(controller: Box2D.Dynamics.Controllers.Controller): void;
        setWarmStarting(flag: bool): void;
        setContinuousPhysics(flag: bool): void;
        getBodyCount(): number;
        getJointCount(): number;
        getContactCount(): number;
        setGravity(gravity: Box2D.Common.Math.Vec2): void;
        getGravity(): Box2D.Common.Math.Vec2;
        getGroundBody(): Box2D.Dynamics.Body;
        step(dt: number, velocityIterations: number, positionIterations: number): void;
        clearForces(): void;
        drawDebugData(): void;
        queryAABB(callback: (d: any) => bool, aabb: Box2D.Collision.AABB): void;
        queryShape(callback: (f: Fixture) => bool, shape: Box2D.Collision.Shapes.Shape, transform?: Box2D.Common.Math.Transform): void;
        queryPoint(callback: (f: Fixture) => bool, p: Box2D.Common.Math.Vec2): void;
        rayCast(callback: (f: Fixture, p: Box2D.Common.Math.Vec2, n: Box2D.Common.Math.Vec2, r: number) => number, point1: Box2D.Common.Math.Vec2, point2: Box2D.Common.Math.Vec2): void;
        rayCastOne(point1: Box2D.Common.Math.Vec2, point2: Box2D.Common.Math.Vec2): Box2D.Dynamics.Fixture;
        rayCastAll(point1: Box2D.Common.Math.Vec2, point2: Box2D.Common.Math.Vec2): Box2D.Dynamics.Fixture[];
        getBodyList(): Box2D.Dynamics.Body;
        getJointList(): Box2D.Dynamics.Joints.Joint;
        getContactList(): Box2D.Dynamics.Contacts.Contact;
        isLocked(): bool;
    }
}
module Box2D.Dynamics.Contacts {
    export class Contact {
        getManifold(): Box2D.Collision.Manifold;
        getWorldManifold(worldManifold: Box2D.Collision.WorldManifold): void;
        isTouching(): bool;
        isContinuous(): bool;
        setSensor(sensor: bool): void;
        isSensor(): bool;
        setEnabled(flag: bool): void;
        isEnabled(): bool;
        getNext(): Box2D.Dynamics.Contacts.Contact;
        getFixtureA(): Box2D.Dynamics.Fixture;
        getFixtureB(): Box2D.Dynamics.Fixture;
        flagForFiltering(): void;
        constructor();
    }
    export class ContactConstraint {
        points: Box2D.Dynamics.Contacts.ContactConstraintPoint[];
        localPlaneNormal: Box2D.Common.Math.Vec2;
        localPoint: Box2D.Common.Math.Vec2;
        normal: Box2D.Common.Math.Vec2;
        normalMass: Box2D.Common.Math.Mat22;
        K: Box2D.Common.Math.Mat22;
        bodyA: Box2D.Dynamics.Body;
        bodyB: Box2D.Dynamics.Body;
        type: number;
        radius: number;
        friction: number;
        restitution: number;
        pointCount: number;
        manifold: Box2D.Collision.Manifold;
        constructor();
    }
    export class ContactConstraintPoint {
        localPoint: Box2D.Common.Math.Vec2;
        rA: Box2D.Common.Math.Vec2;
        rB: Box2D.Common.Math.Vec2;
        normalImpulse: number;
        tangentImpulse: number;
        normalMass: number;
        tangentMass: number;
        equalizedMass: number;
        velocityBias: number;
    }
    export class ContactEdge {
        other: Box2D.Dynamics.Body;
        contact: Box2D.Dynamics.Contacts.Contact;
        prev: Box2D.Dynamics.Contacts.ContactEdge;
        next: Box2D.Dynamics.Contacts.ContactEdge;
    }
    export class ContactFactory {
        create(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): Box2D.Dynamics.Contacts.Contact;
        destroy(contact: Box2D.Dynamics.Contacts.Contact): void;
    }
    export class ContactRegister {
        createFcn: () => any;
        destroyFcn: () => any;
        primary: bool;
        pool: Box2D.Dynamics.Contacts.Contact;
        poolCount: number;
    }
    export class ContactResult {
        shape1: Box2D.Collision.Shapes.Shape;
        shape2: Box2D.Collision.Shapes.Shape;
        position: Box2D.Common.Math.Vec2;
        normal: Box2D.Common.Math.Vec2;
        normalImpulse: number;
        tangentImpulse: number;
        id: Box2D.Collision.ContactID;
    }
    export class ContactSolver {
        constructor();
        initialize(step: Box2D.Dynamics.TimeStep, contacts: Box2D.Dynamics.Contacts.Contact[], contactCount: number, allocator: any): void;
        initVelocityConstraints(step: Box2D.Dynamics.TimeStep): void;
        solveVelocityConstraints(): void;
        finalizeVelocityConstraints(): void;
        solvePositionConstraints(baumgarte: number): bool;
    }
    export class CircleContact extends Contact {
        static create(allocator: any): Box2D.Dynamics.Contacts.Contact;
        static destroy(contact: Box2D.Dynamics.Contacts.Contact, allocator: any): void;
        reset(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): void;
    }
    export class EdgeAndCircleContact extends Contact {
        static create(allocator: any): Box2D.Dynamics.Contacts.Contact;
        static destroy(contact: Box2D.Dynamics.Contacts.Contact, allocator: any): void;
        reset(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): void;
    }
    export class NullContact extends Contact {
        constructor();
    }
    export class PolyAndCircleContact extends Contact {
        static create(allocator: any): Box2D.Dynamics.Contacts.Contact;
        static destroy(contact: Box2D.Dynamics.Contacts.Contact, allocator: any): void;
        reset(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): void;
    }
    export class PolyAndEdgeContact extends Contact {
        static create(allocator: any): Box2D.Dynamics.Contacts.Contact;
        static destroy(contact: Box2D.Dynamics.Contacts.Contact, allocator: any): void;
        reset(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): void;
    }
    export class PolygonContact extends Contact {
        static create(allocator: any): Box2D.Dynamics.Contacts.Contact;
        static destroy(contact: Box2D.Dynamics.Contacts.Contact, allocator: any): void;
        reset(fixtureA: Box2D.Dynamics.Fixture, fixtureB: Box2D.Dynamics.Fixture): void;
    }
}
module Box2D.Dynamics.Controllers {
    export class Controller {
        step(step: Box2D.Dynamics.TimeStep): void;
        draw(debugDraw: Box2D.Dynamics.DebugDraw): void;
        addBody(body: Box2D.Dynamics.Body): void;
        removeBody(body: Box2D.Dynamics.Body): void;
        clear(): void;
        getNext(): Box2D.Dynamics.Controllers.Controller;
        getWorld(): Box2D.Dynamics.World;
        getBodyList(): Box2D.Dynamics.Controllers.ControllerEdge;
    }
    export class BuoyancyController extends Controller {
        normal: Box2D.Common.Math.Vec2;
        offset: number;
        density: number;
        velocity: Box2D.Common.Math.Vec2;
        linearDrag: number;
        angularDrag: number;
        useDensity: bool;
        useWorldGravity: bool;
        gravity: Box2D.Common.Math.Vec2;
        step(step: Box2D.Dynamics.TimeStep): void;
        draw(debugDraw: Box2D.Dynamics.DebugDraw): void;
    }
    export class ConstantAccelController extends Controller {
        A: Box2D.Common.Math.Vec2;
        step(step: Box2D.Dynamics.TimeStep): void;
    }
    export class ConstantForceController extends Controller {
        F: Box2D.Common.Math.Vec2;
        step(step: Box2D.Dynamics.TimeStep): void;
    }
    export class ControllerEdge {
        controller: Box2D.Dynamics.Controllers.Controller;
        body: Box2D.Dynamics.Body;
        prevBody: Box2D.Dynamics.Controllers.ControllerEdge;
        nextBody: Box2D.Dynamics.Controllers.ControllerEdge;
        prevController: Box2D.Dynamics.Controllers.ControllerEdge;
        nextController: Box2D.Dynamics.Controllers.ControllerEdge;
    }
    export class GravityController extends Controller {
        G: number;
        invSqr: bool;
        step(step: Box2D.Dynamics.TimeStep): void;
    }
    export class TensorDampingController extends Controller {
        T: Box2D.Common.Math.Mat22;
        maxTimestep: number;
        setAxisAligned(xDamping: number, yDamping: number): void;
        step(step: Box2D.Dynamics.TimeStep): void;
    }
}
module Box2D.Dynamics.Joints {
    export class Jacobian {
        linearA: Box2D.Common.Math.Vec2;
        angularA: number;
        linearB: Box2D.Common.Math.Vec2;
        angularB: number;
        setZero(): void;
        set(x1: Box2D.Common.Math.Vec2, a1: number, x2: Box2D.Common.Math.Vec2, a2: number): void;
        compute(x1: Box2D.Common.Math.Vec2, a1: number, x2: Box2D.Common.Math.Vec2, a2: number): number;
    }
    export class Joint {
        getType(): number;
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getBodyA(): Box2D.Dynamics.Body;
        getBodyB(): Box2D.Dynamics.Body;
        getNext(): Box2D.Dynamics.Joints.Joint;
        getUserData(): any;
        setUserData(data: any): void;
        isActive(): bool;
        constructor(def: Box2D.Dynamics.Joints.JointDef);
    }
    export class JointDef {
        type: number;
        userData: any;
        bodyA: Box2D.Dynamics.Body;
        bodyB: Box2D.Dynamics.Body;
        collideConnected: bool;
        constructor();
    }
    export class DistanceJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getLength(): number;
        setLength(length: number): void;
        getFrequency(): number;
        setFrequency(hz: number): void;
        getDampingRatio(): number;
        setDampingRatio(ratio: number): void;
        constructor(def: Box2D.Dynamics.Joints.DistanceJointDef);
    }
    export class GearJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getRatio(): number;
        setRatio(ratio: number): void;
        constructor(def: Box2D.Dynamics.Joints.GearJointDef);
    }
    export class FrictionJoint extends Joint {
        m_linearMass: Box2D.Common.Math.Mat22;
        m_angularMass: number;
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        setMaxForce(force: number): void;
        getMaxForce(): number;
        setMaxTorque(torque: number): void;
        getMaxTorque(): number;
        constructor(def: Box2D.Dynamics.Joints.FrictionJointDef);
    }
    export class GearJointDef extends JointDef {
        joint1: Box2D.Dynamics.Joints.Joint;
        joint2: Box2D.Dynamics.Joints.Joint;
        ratio: number;
        constructor();
    }
    export class FrictionJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        maxForce: number;
        maxTorque: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchor: Box2D.Common.Math.Vec2): void;
    }
    
    export class DistanceJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        length: number;
        frequencyHz: number;
        dampingRatio: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchorA: Box2D.Common.Math.Vec2, anchorB: Box2D.Common.Math.Vec2): void;
    }
    export class JointEdge {
        other: Box2D.Dynamics.Body;
        joint: Box2D.Dynamics.Joints.Joint;
        prev: Box2D.Dynamics.Joints.JointEdge;
        next: Box2D.Dynamics.Joints.JointEdge;
    }
}
module Box2D.Dynamics.Joints {
    export class LineJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getJointTranslation(): number;
        getJointSpeed(): number;
        isLimitEnabled(): bool;
        enableLimit(flag: bool): void;
        getLowerLimit(): number;
        getUpperLimit(): number;
        setLimits(lower: number, upper: number): void;
        isMotorEnabled(): bool;
        enableMotor(flag: bool): void;
        setMotorSpeed(speed: number): void;
        getMotorSpeed(): number;
        setMaxMotorForce(force: number): void;
        getMaxMotorForce(): number;
        getMotorForce(): number;
        constructor(def: Box2D.Dynamics.Joints.LineJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class LineJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        localAxisA: Box2D.Common.Math.Vec2;
        enableLimit: bool;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: bool;
        maxMotorForce: number;
        motorSpeed: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchor: Box2D.Common.Math.Vec2, axis: Box2D.Common.Math.Vec2): void;
    }
}
module Box2D.Dynamics.Joints {
    export class MouseJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getTarget(): Box2D.Common.Math.Vec2;
        setTarget(target: Box2D.Common.Math.Vec2): void;
        getMaxForce(): number;
        setMaxForce(maxForce: number): void;
        getFrequency(): number;
        setFrequency(hz: number): void;
        getDampingRatio(): number;
        setDampingRatio(ratio: number): void;
        constructor(def: Box2D.Dynamics.Joints.MouseJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class MouseJointDef extends JointDef {
        target: Box2D.Common.Math.Vec2;
        maxForce: number;
        frequencyHz: number;
        dampingRatio: number;
        constructor();
    }
}
module Box2D.Dynamics.Joints {
    export class PrismaticJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getJointTranslation(): number;
        getJointSpeed(): number;
        isLimitEnabled(): bool;
        enableLimit(flag: bool): void;
        getLowerLimit(): number;
        getUpperLimit(): number;
        setLimits(lower: number, upper: number): void;
        isMotorEnabled(): bool;
        enableMotor(flag: bool): void;
        setMotorSpeed(speed: number): void;
        getMotorSpeed(): number;
        setMaxMotorForce(force: number): void;
        getMotorForce(): number;
        constructor(def: Box2D.Dynamics.Joints.PrismaticJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class PrismaticJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        localAxisA: Box2D.Common.Math.Vec2;
        referenceAngle: number;
        enableLimit: bool;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: bool;
        maxMotorForce: number;
        motorSpeed: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchor: Box2D.Common.Math.Vec2, axis: Box2D.Common.Math.Vec2): void;
    }
}
module Box2D.Dynamics.Joints {
    export class PulleyJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getGroundAnchorA(): Box2D.Common.Math.Vec2;
        getGroundAnchorB(): Box2D.Common.Math.Vec2;
        getLength1(): number;
        getLength2(): number;
        getRatio(): number;
        constructor(def: Box2D.Dynamics.Joints.PulleyJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class PulleyJointDef extends JointDef {
        groundAnchorA: Box2D.Common.Math.Vec2;
        groundAnchorB: Box2D.Common.Math.Vec2;
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        lengthA: number;
        maxLengthA: number;
        lengthB: number;
        maxLengthB: number;
        ratio: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, gaA: Box2D.Common.Math.Vec2, gaB: Box2D.Common.Math.Vec2, anchorA: Box2D.Common.Math.Vec2, anchorB: Box2D.Common.Math.Vec2, r: number): void;
    }
}
module Box2D.Dynamics.Joints {
    export class RevoluteJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        getJointAngle(): number;
        getJointSpeed(): number;
        isLimitEnabled(): bool;
        enableLimit(flag: bool): void;
        getLowerLimit(): number;
        getUpperLimit(): number;
        setLimits(lower: number, upper: number): void;
        isMotorEnabled(): bool;
        enableMotor(flag: bool): void;
        setMotorSpeed(speed: number): void;
        getMotorSpeed(): number;
        setMaxMotorTorque(torque: number): void;
        getMotorTorque(): number;
        constructor(def: Box2D.Dynamics.Joints.RevoluteJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class RevoluteJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        referenceAngle: number;
        enableLimit: bool;
        lowerAngle: number;
        upperAngle: number;
        enableMotor: bool;
        motorSpeed: number;
        maxMotorTorque: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchor: Box2D.Common.Math.Vec2): void;
    }
}
module Box2D.Dynamics.Joints {
    export class WeldJoint extends Joint {
        getAnchorA(): Box2D.Common.Math.Vec2;
        getAnchorB(): Box2D.Common.Math.Vec2;
        getReactionForce(inv_dt: number): Box2D.Common.Math.Vec2;
        getReactionTorque(inv_dt: number): number;
        constructor(def: Box2D.Dynamics.Joints.WeldJointDef);
    }
}
module Box2D.Dynamics.Joints {
    export class WeldJointDef extends JointDef {
        localAnchorA: Box2D.Common.Math.Vec2;
        localAnchorB: Box2D.Common.Math.Vec2;
        referenceAngle: number;
        constructor();
        initialize(bA: Box2D.Dynamics.Body, bB: Box2D.Dynamics.Body, anchor: Box2D.Common.Math.Vec2): void;
    }
}
