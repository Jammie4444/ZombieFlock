using UnityEngine;
using System.Collections.Generic;

public class ZombieSwarm : MonoBehaviour
{
    public GameObject player;
    public float speed = 2.0f;
    public float separationDistance = 1.5f; // Minimum distance between zombies
    public float cohesionStrength = 0.1f; // How strongly zombies stick together
    public float alignmentStrength = 0.2f; // How strongly zombies align their direction
    public float avoidanceStrength = 2.0f; // How strongly zombies avoid obstacles
    public float avoidanceDistance = 2.5f;  // Distance at which zombies start avoiding obstacles
    public float attackRange = 1.0f; // Distance at which the zombie can attack the player.
    public LayerMask obstacleMask; // Layer mask for obstacles (walls, etc.)
    public LayerMask playerLayer;   //  Layer for the player
    public LayerMask zombieLayer; // Layer that the zombies are on.


    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Ensure the zombie has a Rigidbody.  If it doesn't, add one.
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ; // Keep upright.

        // Ensure the required components exist
        if (player == null)
        {
            Debug.LogError("Player object not assigned to ZombieSwarm script!");
            enabled = false; // Disable the script if critical components are missing.
            return;
        }
    }


    void FixedUpdate()  // Use FixedUpdate for physics-based movement
    {
        if (player == null) return; // Safety check

        Vector3 desiredVelocity = Vector3.zero;

        // 1. Seek the Player
        Vector3 toPlayer = player.transform.position - transform.position;
        float distanceToPlayer = toPlayer.magnitude;

        if (distanceToPlayer > attackRange)
        {
            desiredVelocity += toPlayer.normalized * speed;
        }

        // 2. Separation (Avoid crowding other zombies)
        Vector3 separationForce = Vector3.zero;
        int neighborCount = 0;

        // Get nearby zombies efficiently using OverlapSphere.  This is MUCH faster than iterating through all zombies.
        Collider[] nearbyZombies = Physics.OverlapSphere(transform.position, separationDistance * 2f, zombieLayer); // *2f to give some buffer
        foreach (Collider zombieCollider in nearbyZombies)
        {
            if (zombieCollider.gameObject != gameObject) // Don't compare against self.
            {
                Vector3 difference = transform.position - zombieCollider.transform.position;
                float distance = difference.magnitude;

                if (distance < separationDistance)
                {
                    // Scale the separation force inversely by distance. Closer = stronger force.
                    separationForce += difference.normalized / Mathf.Max(distance, 0.1f); // Avoid division by zero.
                    neighborCount++;
                }
            }
        }

        if (neighborCount > 0)
        {
            separationForce /= neighborCount;  // Average the separation force
            desiredVelocity += separationForce * avoidanceStrength;
        }


        // 3. Cohesion (Move towards the average position of nearby zombies)
        Vector3 cohesionForce = Vector3.zero;
        neighborCount = 0; // Reset neighbor count

        foreach (Collider zombieCollider in nearbyZombies)
        {
            if (zombieCollider.gameObject != gameObject)
            {
                cohesionForce += zombieCollider.transform.position;
                neighborCount++;
            }
        }

        if (neighborCount > 0)
        {
            cohesionForce /= neighborCount;
            cohesionForce = (cohesionForce - transform.position).normalized; // Direction towards center
            desiredVelocity += cohesionForce * cohesionStrength;
        }

        // 4. Alignment (Match direction with nearby zombies)
        Vector3 alignmentForce = Vector3.zero;
        neighborCount = 0;  // Reset
        foreach (Collider zombieCollider in nearbyZombies)
        {
           if (zombieCollider.gameObject != gameObject)
            {
             Rigidbody otherZombieRb = zombieCollider.GetComponent<Rigidbody>();
             if (otherZombieRb != null) // Check in case another zombie doesn't have an rb
             {
                alignmentForce += otherZombieRb.linearVelocity; // Use velocity for alignment
                neighborCount++;
             }
            }
        }

        if (neighborCount > 0)
        {
            alignmentForce /= neighborCount;
            alignmentForce = alignmentForce.normalized;
            desiredVelocity += alignmentForce * alignmentStrength;
        }



        // 5. Obstacle Avoidance
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, avoidanceDistance, obstacleMask))
        {
            // Calculate a direction away from the obstacle's normal.
            Vector3 avoidanceDirection = Vector3.Reflect(transform.forward, hit.normal);
            desiredVelocity += avoidanceDirection * avoidanceStrength;
        }
        // Also check with raycasts to the sides for better avoidance.
        if (Physics.Raycast(transform.position, (transform.forward + transform.right).normalized, out hit, avoidanceDistance * 0.7f, obstacleMask)) //0.7f is good enough
        {
            Vector3 avoidanceDirection = Vector3.Reflect((transform.forward + transform.right).normalized, hit.normal);
            desiredVelocity += avoidanceDirection * avoidanceStrength;
        }

        if (Physics.Raycast(transform.position, (transform.forward - transform.right).normalized, out hit, avoidanceDistance * 0.7f, obstacleMask))
        {
            Vector3 avoidanceDirection = Vector3.Reflect((transform.forward - transform.right).normalized, hit.normal);
            desiredVelocity += avoidanceDirection * avoidanceStrength;
        }



        // 6. Apply Movement
        // Normalize the desired velocity and apply speed.  This prevents "stacking" of forces.
        desiredVelocity = desiredVelocity.normalized * speed;
        rb.linearVelocity = new Vector3(desiredVelocity.x, rb.linearVelocity.y, desiredVelocity.z); // Preserve vertical velocity (gravity).

        // 7. Rotation (Look at movement direction)
        if (rb.linearVelocity.magnitude > 0.1f) // Avoid snapping to zero rotation
        {
            Quaternion targetRotation = Quaternion.LookRotation(new Vector3(rb.linearVelocity.x, 0, rb.linearVelocity.z));
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 10f); // Smooth rotation
        }

        // 8. Attack check  (Simple example, could be expanded)
         if (distanceToPlayer <= attackRange)
        {
            // Perform attack action here (e.g., reduce player health, play animation)
            Debug.Log("Zombie attacks!"); // Replace with actual attack logic.
        }
    }

    //Draw gizmos to show debug information
    void OnDrawGizmosSelected()
    {
        //Separation Distance
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, separationDistance);

        //Avoidance Distance
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, avoidanceDistance);

        //Attack Range
        Gizmos.color = Color.magenta;
        Gizmos.DrawWireSphere(transform.position, attackRange);

        //Forward direction
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * 2);
    }
}