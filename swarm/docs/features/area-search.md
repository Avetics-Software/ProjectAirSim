# Area search

Draw a search area once; the ground station splits it between the drones,
gives each a PX4 mission and follows their progress on the map.

## Using it

**Dashboard:** turn on *Draw area*, click the corners on the minimap (3 or
more, any shape, concave is fine), set *Height*, *Pass spacing* and *Drones*
(`all`, or e.g. `1,3`), and press *Search area*.

**Console:**

```
1,3 searcharea 30 40 1.4140,103.8650;1.4160,103.8650;1.4160,103.8670
all search 1.4140 103.8650 1.4160 103.8670 30 40     # a lat/lon box
```

The arguments are the height (m above home), the pass spacing (m) and the
corners.

## How the area is split

`gcs/search_planner.py`, `plan_area_search()`:

1. **Passes along the longest edge.** Parallel passes `spacing` metres apart
   run along the area's longest edge, so the drones turn as rarely as possible.
2. **Concave areas.** A pass that crosses the area more than once (a U shape,
   say) becomes several pieces, so no drone flies over ground outside it.
3. **One block per drone.** The passes are split into contiguous blocks, one
   per drone, balanced by length (passes plus the turns between them), so the
   drone with the most to fly finishes as early as possible. Contiguous blocks
   keep the drones in separate parts of the area.
4. **Back and forth.** Each drone flies its block as a lawnmower pattern.

A drone whose share would be empty (a small area, many drones) is left out
and told so.

## Flying it

`gcs/swarm.py`, `search_area()`:

- Each drone's block is uploaded to its PX4 as a mission (with a takeoff item
  if it is on the ground), at its own [height layer](deconfliction.md), and
  started. The mission lives on the drone: it keeps flying it if the ground
  station's link drops, and QGC shows it.
- The new routes are [checked for crossings](deconfliction.md) with the
  other drones' routes.
- The ground station follows each drone's mission progress and draws its
  remaining route on the minimap. When a drone finishes, it holds at its last
  waypoint.
- A new command for a drone replaces its search.

## Choosing the spacing

The spacing should be no wider than what the camera sees across the ground,
or the passes leave gaps. Looking straight down, a camera with horizontal
field of view *HFOV* at height *h* covers about `2 h tan(HFOV / 2)` across:
with the sim's camera at 1x (61.5°), about 36 m at 30 m and 60 m at 50 m. Zoom
narrows it. Choosing the spacing from the camera and height automatically is
on the [roadmap](../roadmap.md).
