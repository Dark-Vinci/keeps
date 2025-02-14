    /**
     * A particle is the simplest object that can be simulated in the
     * physics system.

     */

#ifndef PARTICLE_H
#define PARTICLE_H

#include "coreMath.h"
#include <array>
#include <vector>

    class Particle
    {
    protected:

	float inverseMass;
	float damping;
	float radius;
    Vector2 position;
	Vector2 velocity;
	Vector2 forceAccum;
	Vector2 acceleration;
	std::array<float, 3> color;
    
	public:
		void setColor(const float x, const float y, const float z);
		std::array<float, 3> getColor();
		void integrate(float duration);
		void setMass(const float mass);
		float getMass() const;
		void setInverseMass(const float inverseMass);
		float getInverseMass() const;
		bool hasFiniteMass() const;

	    void setDamping(const float damping);
        float getDamping() const;

        void setPosition(const float x, const float y);
		void setPosition(const Vector2 &position);
		Vector2 getPosition() const;
		void getPosition(Vector2 *position) const;
		
		void setRadius(const float r);
		float getRadius() const;
		
		void setVelocity(const Vector2 &velocity);
		void setVelocity(const float x, const float y);
		Vector2 getVelocity() const;
		void getVelocity(Vector2 *velocity) const;

		void setAcceleration(const Vector2 &acceleration);
		void setAcceleration(const float x, const float y);
		Vector2 getAcceleration() const;

		void clearAccumulator();
		void addForce(const Vector2 &force);
	
       };

	#endif // 


