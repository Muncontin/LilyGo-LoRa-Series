#include <Arduino.h>

#define MAX_ROUTES 20  // Max number of routing table entries

struct RouteEntry {
    String destination;  // Destination Node ID
    String nextHop;      // Next Hop Node ID
    int distance;        // Hop count to destination
    int spreadingFactor; // SF used for this route
    unsigned long expiryTime; // Expiration time for the route
};

RouteEntry routingTable[MAX_ROUTES];  // Routing table
int routeCount = 0;  // Current number of routes
