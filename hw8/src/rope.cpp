#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D distance = (end-start)/(num_nodes-1);
        Mass* last_m;
        for(int i=0;i<num_nodes;i++)
        {
            Mass* m = new Mass(start+distance*i,node_mass,false);
            masses.push_back(m);
            if(i>0)
            {
                Spring* s = new Spring(last_m,m,k);
                springs.push_back(s);
            }
            last_m = m;
        }

        for(auto &i:pinned_nodes){
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a = s->m1->position;
            Vector2D b = s->m2->position;
            float k = s->k;
            double l = s->rest_length;
            Vector2D f = k*(b-a).unit()*((b-a).norm()-l); // pull a to b
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity*m->mass;
                // TODO (Part 2): Add global damping
                float kd = 0.005; // damping coefficient，自己调节
                m->forces += -kd*m->velocity;
                Vector2D a = m->forces/m->mass;
                // 显式欧拉法
              /*  m->position += m->velocity * delta_t;
                m->velocity += a * delta_t;*/
                // 半隐式欧拉法
                m->velocity += a*delta_t;
                m->position += m->velocity*delta_t; 
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        bool KisInfinite = true; // 弹簧劲度系数为无穷大。应该针对每段弹簧的k定义（比如-1表示无穷大）
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            if(!KisInfinite)
            {
                Vector2D a = s->m1->position;
                Vector2D b = s->m2->position;
                float k = s->k;
                double l = s->rest_length;
                Vector2D f = k*(b-a).unit()*((b-a).norm()-l); // pull a to b
                s->m1->forces += f;
                s->m2->forces += -f;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity*m->mass;
                Vector2D a = m->forces/m->mass;
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005;
                m->position += (1-damping_factor)*(m->position-m->last_position)+a*delta_t*delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }

        // 不用再考虑弹簧力，而是用解约束的方法来更新质点位置：
        // 只要简单的移动每个质点的位置使得弹簧的长度保持原长。
        // 修正向量应该和两个质点之间的位移成比例，方向为一个质点指向另一质点。每个质点应该移动位移的一半。
        if(KisInfinite)
        {
            for (auto &s : springs)
            {
                double distance = (s->m2->position - s->m1->position).norm();
                Vector2D dir = (s->m2->position - s->m1->position).unit();
                if(!s->m1->pinned && !s->m2->pinned)
                {
                    s->m1->position += 0.5*(distance - s->rest_length)*dir;
                    s->m2->position += -0.5*(distance - s->rest_length)*dir;
                }
                else if(!s->m1->pinned)
                    s->m1->position += (distance - s->rest_length)*dir;
                else if(!s->m2->pinned)
                    s->m2->position += -(distance - s->rest_length)*dir;
            }
        }
    }
}
