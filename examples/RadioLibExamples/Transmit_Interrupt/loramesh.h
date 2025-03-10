#include <Arduino.h>

#define MAX_ROUTES 20  // Max number of routing table entries

struct RouteEntry {
    String destination;  // Destination Node ID
    String nextHop;      // Next hop towards destination
    int hopCount;        // Number of hops to reach destination
    unsigned long expiryTime; // Timestamp to remove old routes
};

RouteEntry routingTable[MAX_ROUTES];  // Routing table
int routeCount = 0;  // Current number of routes
