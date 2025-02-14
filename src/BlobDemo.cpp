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
#include <cassert>

constexpr auto PLATFORM_COUNT = 7;
constexpr auto BLOB_COUNT = 30;

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

// GENERATES RANDOM FLOAT VALUE IN RANGE MIN <= VAL <= MAX
static float getRandomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}

// TEST FOR getRandomFloat FUNCTION
void testGetRandomFloat() {
    for (int i = 0; i < 1000; ++i) {
        float value = getRandomFloat(5.0f, 10.0f);
        assert(value >= 5.0f && value <= 10.0f);
    }
}


// Struct representing an Axis-Aligned Bounding Box (AABB)
struct AABB {
    float x, y, width, height; // Position (x, y) and dimensions (width, height)

    // Checks if a given point is inside the bounding box
    bool contains(const Vector2& point) const {
        return (point.x >= x && point.x <= x + width &&  // Point within x range
            point.y >= y && point.y <= y + height);  // Point within y range
    }

    // Checks if this AABB intersects with another AABB
    bool intersects(const AABB& other) const {
        // No intersection if one box is entirely to the left, right, above, or below the other
        return !(other.x > x + width ||        // Other box is completely to the right
            other.x + other.width < x ||  // Other box is completely to the left
            other.y > y + height ||       // Other box is completely below
            other.y + other.height < y);  // Other box is completely above
    }
};


// Class representing a Quadtree for spatial partitioning
class Quadtree {
private:
    static constexpr int MAX_OBJECTS = 4; // Maximum objects a node can hold before subdividing
    static constexpr int MAX_LEVELS = 5;  // Maximum depth of the tree

    int level;                 // Current depth level of this node
    std::vector<Vector2> objects; // Objects contained in this node
    AABB bounds;               // The bounding box representing this region
    std::unique_ptr<Quadtree> children[4]; // Four child nodes (subdivisions)
    bool divided = false;      // Flag to indicate if this node has been subdivided

public:
    // Constructor: Initializes a Quadtree node with level and boundary
    Quadtree(int lvl, AABB boundary) : level(lvl), bounds(boundary) {}

    // Subdivides the current node into four child nodes
    void subdivide() {
        float halfWidth = bounds.width / 2.0f;
        float halfHeight = bounds.height / 2.0f;
        float x = bounds.x;
        float y = bounds.y;

        // Creating four child nodes (Top-Left, Top-Right, Bottom-Left, Bottom-Right)
        children[0] = std::make_unique<Quadtree>(level + 1, AABB{ x, y, halfWidth, halfHeight });       // Top-left
        children[1] = std::make_unique<Quadtree>(level + 1, AABB{ x + halfWidth, y, halfWidth, halfHeight });  // Top-right
        children[2] = std::make_unique<Quadtree>(level + 1, AABB{ x, y + halfHeight, halfWidth, halfHeight }); // Bottom-left
        children[3] = std::make_unique<Quadtree>(level + 1, AABB{ x + halfWidth, y + halfHeight, halfWidth, halfHeight }); // Bottom-right
        divided = true; // Mark node as subdivided
    }

    // Inserts a point into the quadtree
    void insert(const Vector2& point) {
        // If the point is outside the boundary, ignore it
        if (!bounds.contains(point)) return;

        // If there is room in this node, add the point
        if (objects.size() < MAX_OBJECTS) {
            objects.push_back(point);
        }
        else {
            // Subdivide if not already done
            if (!divided) subdivide();

            // Insert the point into the appropriate child node
            for (auto& child : children) {
                child->insert(point);
            }
        }
    }

    // Queries the quadtree for objects within a given range
    void query(const AABB& range, std::vector<Vector2>& found) const {
        // If the query range does not intersect this node, return
        if (!bounds.intersects(range)) return;

        // Check if objects in this node are within the query range
        for (const auto& obj : objects) {
            if (range.contains(obj)) {
                found.push_back(obj);
            }
        }

        // Recursively check child nodes if they exist
        if (divided) {
            for (const auto& child : children) {
                child->query(range, found);
            }
        }
    }

    // Draws the quadtree boundaries using OpenGL
    void draw() const {
        // Set color for drawing (magenta)
        glColor3f(1.0f, 0.0f, 1.0f);

        // Draw the bounding box of this node
        glBegin(GL_LINE_LOOP);
        glVertex2f(bounds.x, bounds.y);
        glVertex2f(bounds.x + bounds.width, bounds.y);
        glVertex2f(bounds.x + bounds.width, bounds.y + bounds.height);
        glVertex2f(bounds.x, bounds.y + bounds.height);
        glEnd();

        // Recursively draw child nodes if subdivided
        if (divided) {
            for (const auto& child : children) {
                child->draw();
            }
        }
    }
};


// Represents a platform that can generate contacts when a particle collides with it
class Platform : public ParticleContactGenerator {
public:
    Vector2 start;   // Start position of the platform (one endpoint)
    Vector2 end;     // End position of the platform (other endpoint)
    float restitution; // Restitution coefficient (elasticity of the collision)

    /**
     * Holds a pointer to the particle we're checking for collisions with.
     * This allows the platform to detect and resolve collisions with a specific particle.
     */
    Particle* particle;

    /**
     * Generates a contact if the particle collides with the platform.
     *
     * @param contact A pointer to the array of contacts where the generated contact will be stored.
     * @param limit The maximum number of contacts that can be added.
     * @return The number of contacts generated (0 or 1, since a particle can collide with the platform at most once).
     */
    virtual unsigned addContact(
        ParticleContact* contact,
        unsigned limit
    ) const;
};


unsigned Platform::addContact(ParticleContact* contact, unsigned limit) const
{
    const static float restitution = 1.0f; // Coefficient of restitution (elastic collision)
    unsigned used = 0; // Keeps track of the number of contacts generated

    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        if (used >= limit) return used; // Ensure we don't exceed the contact limit

        // Vector from platform's start point to the particle
        Vector2 toParticle = particle[i].getPosition() - start;
        Vector2 lineDirection = end - start; // Direction vector of the platform

        // Project the particle's position onto the platform
        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude(); // Squared length of the platform
        float squareRadius = particle[i].getRadius() * particle[i].getRadius(); // Squared radius of the particle

        if (projected <= 0)
        {
            // The particle is closest to the start point of the platform
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // Collision detected
                contact->contactNormal = toParticle.unit(); // Normalized collision direction
                contact->restitution = restitution; // Elastic collision response
                contact->particle[0] = &particle[i]; // Assign the colliding particle
                contact->particle[1] = nullptr; // No second particle involved
                contact->penetration = particle[i].getRadius() - toParticle.magnitude(); // Overlap distance
                used++; // Increment contact count
                contact++; // Move to the next contact in the array
            }
        }
        else if (projected >= platformSqLength)
        {
            // The particle is closest to the end point of the platform
            toParticle = particle[i].getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // Collision detected
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
            // The particle is nearest to the middle of the platform
            float distanceToPlatform = toParticle.squareMagnitude() - (projected * projected / platformSqLength);
            if (distanceToPlatform < squareRadius)
            {
                // Collision detected
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength); // Closest point on the platform

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
    return used; // Return the number of generated contacts
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
    // Set the window dimensions
    width = 400;
    height = 400;

    // Define range and margin for platform positioning
    nRange = 100.0;
    float margin = 1.0;

    // Allocate memory for blob particles
    blob = new Particle[BLOB_COUNT];

    // Allocate memory for platform objects
    platform = new Platform[PLATFORM_COUNT];

    // Define platforms with start and end positions and restitution values
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

    // Link particles (blobs) to platforms
    for (unsigned i = 0; i < PLATFORM_COUNT; i++) {
        platform[i].particle = blob; // Assign the array of blobs to each platform
        world.getContactGenerators().push_back(platform + i); // Add platform as a contact generator
    }

    // Initialize blob particles
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        // Generate random position within a bounded range
        float x = rand() % 50;  // X position within window width
        float y = rand() % 50;  // Y position within window height
        float radius = 2 + (rand() % 10); // Random radius between 2 and 11

        // Assign random color to each blob
        blob[i].setColor(getRandomFloat(0.0, 1.0), getRandomFloat(0.0, 1.0), getRandomFloat(0.0, 1.0));

        // Set initial position and radius
        blob[i].setPosition(x, y);
        blob[i].setRadius(radius);

        // Set velocity, damping, and acceleration
        blob[i].setVelocity(0, 0);
        blob[i].setDamping((float)0.9);
        blob[i].setAcceleration(Vector2::GRAVITY * 5.0f * (i + 1)); // Apply gravity scaled by index
        blob[i].setMass(100.0f); // Set mass of blob particle

        // Clear accumulated forces
        blob[i].clearAccumulator();

        // Insert blob's position into quadtree for spatial partitioning
        quadtree.insert(blob[i].getPosition());

        // Add blob to physics world
        world.getParticles().push_back(blob + i);
    }
}


BlobDemo::~BlobDemo()
{
    delete blob;
}

void BlobDemo::display()
{
    // Call base class display function
    Application::display();

    // Draw platforms as lines
    glBegin(GL_LINES);
    glColor3f(0, 1, 1); // Set color to cyan
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        const Vector2& p0 = platform[i].start;
        const Vector2& p1 = platform[i].end;
        glVertex2f(p0.x, p0.y);
        glVertex2f(p1.x, p1.y);
    }
    glEnd();

    // Draw particles (blobs) as spheres
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        // Retrieve blob color and set it
        std::array<float, 3> color = blob[i].getColor();
        glColor3f(color[0], color[1], color[2]);

        // Get blob position
        const Vector2& p = blob[i].getPosition();
        glPushMatrix();
        glTranslatef(p.x, p.y, 0); // Move to blob position

        // Alternate sphere detail for rendering performance
        if (i % 2 == 0) {
            glutSolidSphere(blob[i].getRadius(), 360, 12); // Higher resolution sphere
        }
        else {
            glutSolidSphere(blob[i].getRadius(), 8, 12); // Lower resolution sphere
        }

        glPopMatrix(); // Restore transformation state
    }

    // Draw the quadtree visualization
    quadtree.draw();

    // Swap buffers to update the display
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
    // Convert time interval from milliseconds to seconds
    float duration = timeinterval / 1000.0f;

    // Run the physics simulation for the current time step
    world.runPhysics(duration);

    // Reset the quadtree to prepare for a new frame of spatial partitioning
    quadtree = Quadtree(0, AABB{ -100, -100, 200, 200 });

    // Insert all blob positions into the quadtree for optimized collision detection
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        quadtree.insert(blob[i].getPosition());
    }

    // Handle collisions between blobs
    this->resolveParticleCollisions();

    // Call the base class update function (handles rendering, input, etc.)
    Application::update();
}


const char* BlobDemo::getTitle()
{
    return "Particle collider";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BlobDemo();
}