//Blob Demo
/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include <iostream>
#include <random>
#include <array>
#define PLATFORM_COUNT 7
#define BLOB_COUNT 30

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

float getRandomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}

/**
 * Platforms are two dimensional: lines on which the
 * particles can rest. Platforms are also contact generators for the physics.
 */

struct AABB { // Axis-Aligned Bounding Box
    float x, y, width, height;

    bool contains(const Vector2& point) const {
        return (point.x >= x && point.x <= x + width &&
            point.y >= y && point.y <= y + height);
    }

    bool intersects(const AABB& other) const {
        return !(other.x > x + width || other.x + other.width < x ||
            other.y > y + height || other.y + other.height < y);
    }
};

class Quadtree {
private:
    static constexpr int MAX_OBJECTS = 4; // Max objects before subdividing
    static constexpr int MAX_LEVELS = 5;  // Max depth of the tree

    int level;
    std::vector<Vector2> objects;
    AABB bounds;
    std::unique_ptr<Quadtree> children[4];
    bool divided = false;

public:
    Quadtree(int lvl, AABB boundary) : level(lvl), bounds(boundary) {}

    void subdivide() {
        float halfWidth = bounds.width / 2.0f;
        float halfHeight = bounds.height / 2.0f;
        float x = bounds.x;
        float y = bounds.y;

        children[0] = std::make_unique<Quadtree>(level + 1, AABB{ x, y, halfWidth, halfHeight });       // Top-left
        children[1] = std::make_unique<Quadtree>(level + 1, AABB{ x + halfWidth, y, halfWidth, halfHeight });  // Top-right
        children[2] = std::make_unique<Quadtree>(level + 1, AABB{ x, y + halfHeight, halfWidth, halfHeight }); // Bottom-left
        children[3] = std::make_unique<Quadtree>(level + 1, AABB{ x + halfWidth, y + halfHeight, halfWidth, halfHeight }); // Bottom-right
        divided = true;
    }

    void insert(const Vector2& point) {
        if (!bounds.contains(point)) return;

        if (objects.size() < MAX_OBJECTS) {
            objects.push_back(point);
        }
        else {
            if (!divided) subdivide();

            for (auto& child : children) {
                child->insert(point);
            }
        }
    }

    void query(const AABB& range, std::vector<Vector2>& found) const {
        if (!bounds.intersects(range)) return;

        for (const auto& obj : objects) {
            if (range.contains(obj)) {
                found.push_back(obj);
            }
        }

        if (divided) {
            for (const auto& child : children) {
                child->query(range, found);
            }
        }
    }

    // Draw the quadtree boundaries
    void draw() const {
        // Draw the bounding box of this node
        glColor3f(1.0f, 0.0f, 1.0f); // White color for quadtree boundaries
        glBegin(GL_LINE_LOOP);
        glVertex2f(bounds.x, bounds.y);
        glVertex2f(bounds.x + bounds.width, bounds.y);
        glVertex2f(bounds.x + bounds.width, bounds.y + bounds.height);
        glVertex2f(bounds.x, bounds.y + bounds.height);
        glEnd();

        // Recursively draw children
        if (divided) {
            for (const auto& child : children) {
                child->draw();
            }
        }
    }
};

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    float restitution;
    /**
     * Holds a pointer to the particles we're checking for collisions with.
     */
    Particle* particle;

    virtual unsigned addContact(
        ParticleContact* contact,
        unsigned limit
    ) const;
};

unsigned Platform::addContact(ParticleContact* contact, unsigned limit) const
{
    const static float restitution = 1.0f; // Define restitution for the collision
    unsigned used = 0;

    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        if (used >= limit) return used;

        // Check for penetration
        Vector2 toParticle = particle[i].getPosition() - start;
        Vector2 lineDirection = end - start;

        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle[i].getRadius() * particle[i].getRadius();

        if (projected <= 0)
        {
            // The blob is nearest to the start point
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = &particle[i];
                contact->particle[1] = nullptr;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        else if (projected >= platformSqLength)
        {
            // The blob is nearest to the end point
            toParticle = particle[i].getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = &particle[i];
                contact->particle[1] = nullptr;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        else
        {
            // The blob is nearest to the middle of the platform
            float distanceToPlatform = toParticle.squareMagnitude() - (projected * projected / platformSqLength);
            if (distanceToPlatform < squareRadius)
            {
                // We have a collision
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);

                contact->contactNormal = (particle[i].getPosition() - closestPoint).unit();
                contact->restitution = restitution;
                contact->particle[0] = &particle[i];
                contact->particle[1] = nullptr;
                contact->penetration = particle[i].getRadius() - sqrt(distanceToPlatform);
                used++;
                contact++;
            }
        }
    }
    return used;
}

class BlobDemo : public Application
{
    Particle* blob;

    Platform* platform;

    ParticleWorld world;
    Quadtree quadtree;

public:
    /** Creates a new demo object. */
    BlobDemo();
    virtual ~BlobDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();

    virtual void resolveParticleCollisions();

};

// Method definitions
BlobDemo::BlobDemo() :
    world(PLATFORM_COUNT, PLATFORM_COUNT), quadtree(0, AABB{ 0, 0, 100 + 0.95, 100 + 0.95 })
{
    width = 400; height = 400;
    nRange = 100.0;
    float margin = 1.0;

    // Create blob array
    blob = new Particle[BLOB_COUNT];

    // Create platforms
    platform = new Platform[PLATFORM_COUNT];

    platform[0].start = Vector2(-20.0, 30.0);
    platform[0].end = Vector2(20.0, -10.0);
    platform[0].restitution = getRandomFloat(0.5, 1.0);

    platform[1].start = Vector2(-nRange * margin, -nRange * margin);
    platform[1].end = Vector2(nRange * margin, -nRange * margin);
    platform[1].restitution = getRandomFloat(0.5, 1.0);

    platform[2].start = Vector2(-nRange * margin, nRange * margin);
    platform[2].end = Vector2(nRange * margin, nRange * margin);
    platform[2].restitution = getRandomFloat(0.5, 1.0);

    platform[3].start = Vector2(-nRange * margin, -nRange * margin);
    platform[3].end = Vector2(-nRange * margin, nRange * margin);
    platform[3].restitution = getRandomFloat(0.5, 1.0);

    platform[4].start = Vector2(nRange * margin, -nRange * margin);
    platform[4].end = Vector2(nRange * margin, nRange * margin);
    platform[4].restitution = getRandomFloat(0.5, 1.0);

    platform[5].start = Vector2(80.0, -40.0);
    platform[5].end = Vector2(0.0, -70.0);
    platform[5].restitution = getRandomFloat(0.5, 1.0);

    platform[6].start = Vector2(-20.0, -80.0);
    platform[6].end = Vector2(-80.0, -50.0);
    platform[6].restitution = getRandomFloat(0.5, 1.0);


    // Link particles to platforms
    for (unsigned i = 0; i < PLATFORM_COUNT; i++) {
        platform[i].particle = blob; // Point to the array of blobs
        world.getContactGenerators().push_back(platform + i);
    }


    // Initialize blobs
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        float x = rand() % 50;  // Ensure x is within window width
        float y = rand() % 50; // Ensure y is within window height
        float radius = 2 + (rand() % 10);

        blob[i].setColor(getRandomFloat(0.0, 1.0), getRandomFloat(0.0, 1.0), getRandomFloat(0.0, 1.0));
        blob[i].setPosition(x, y);
        blob[i].setRadius(radius);
        blob[i].setVelocity(0, 0);
        blob[i].setDamping(0.9);
        blob[i].setAcceleration(Vector2::GRAVITY * 5.0f * (i + 1));
        blob[i].setMass(100.0f);
        blob[i].clearAccumulator();
        quadtree.insert(blob[i].getPosition());

        world.getParticles().push_back(blob + i);
    }
}

BlobDemo::~BlobDemo()
{
    delete blob;
}

void BlobDemo::display()
{
    Application::display();

    // Draw platforms
    glBegin(GL_LINES);
    glColor3f(0, 1, 1);
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        const Vector2& p0 = platform[i].start;
        const Vector2& p1 = platform[i].end;
        glVertex2f(p0.x, p0.y);
        glVertex2f(p1.x, p1.y);
    }
    glEnd();

    // Draw particles (blobs)
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        std::array<float, 3> color = blob[i].getColor();
        glColor3f(color[0], color[1], color[2]);
        const Vector2& p = blob[i].getPosition();
        glPushMatrix();
        glTranslatef(p.x, p.y, 0);

        if (i % 2 == 0) {
            glutSolidSphere(blob[i].getRadius(), 360, 12);
        }
        else {
            glutSolidSphere(blob[i].getRadius(), 8, 12);
        }
        glPopMatrix();
    }

    quadtree.draw();

    glutSwapBuffers();
}

void BlobDemo::resolveParticleCollisions()
{
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        // Define a query range around the particle
        AABB queryRange{
            blob[i].getPosition().x - blob[i].getRadius(),
            blob[i].getPosition().y - blob[i].getRadius(),
            2 * blob[i].getRadius(),
            2 * blob[i].getRadius()
        };

        // Query the quadtree for nearby particles
        std::vector<Vector2> nearbyParticles;
        quadtree.query(queryRange, nearbyParticles);

        // Check for collisions with nearby particles
        for (const auto& pos : nearbyParticles)
        {
            for (unsigned j = 0; j < BLOB_COUNT; j++)
            {
                if (i == j) continue; // Skip self

                Particle& p1 = blob[i];
                Particle& p2 = blob[j];

                Vector2 posDiff = p2.getPosition() - p1.getPosition();
                float distance = posDiff.magnitude();
                float radiusSum = p1.getRadius() + p2.getRadius();

                // Check if the particles are overlapping
                if (distance < radiusSum)
                {
                    // Collision detected, compute penetration
                    float penetration = radiusSum - distance;

                    // Compute normal (direction from p1 to p2)
                    Vector2 collisionNormal = posDiff.unit();

                    // Separate the particles so they don't overlap
                    float totalMass = p1.getMass() + p2.getMass();
                    float moveAmount1 = (penetration * (p2.getMass() / totalMass));
                    float moveAmount2 = (penetration * (p1.getMass() / totalMass));

                    p1.setPosition(p1.getPosition() - collisionNormal * moveAmount1);
                    p2.setPosition(p2.getPosition() + collisionNormal * moveAmount2);

                    // Compute relative velocity
                    Vector2 relativeVelocity = p2.getVelocity() - p1.getVelocity();
                    float velocityAlongNormal = relativeVelocity * collisionNormal;

                    // If moving away, ignore response
                    if (velocityAlongNormal > 0) continue;

                    // Compute restitution (bounciness)
                    float restitution = 0.8f;

                    // Compute impulse scalar
                    float impulseScalar = -(1 + restitution) * velocityAlongNormal / (1 / p1.getMass() + 1 / p2.getMass());

                    // Compute impulse vector
                    Vector2 impulse = collisionNormal * impulseScalar;

                    // Apply impulse
                    p1.setVelocity(p1.getVelocity() - impulse * (1 / p1.getMass()));
                    p2.setVelocity(p2.getVelocity() + impulse * (1 / p2.getMass()));
                }
            }
        }
    }
}

void BlobDemo::update()
{
    // Recenter the axes
    float duration = timeinterval / 1000.0f;

    // Run the simulation
    world.runPhysics(duration);

    quadtree = Quadtree(0, AABB{ -100, -100, 200, 200 }); // Reset quadtree
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        quadtree.insert(blob[i].getPosition());
    }

    this->resolveParticleCollisions();

    // Update the application (e.g., render or input handling)
    Application::update();
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BlobDemo();
}