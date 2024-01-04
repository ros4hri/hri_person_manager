
Notes:

- anonymous nodes must start with `anon`
- the main graph is kept and updated across tests steps

THRESHOLD: 0.4

# Step 1

## INPUT

```
body3,person2,0.7
person2,face1,0.9
face1,body2,0.8
face1,person1,0.2
face1,body1,0.1
person1,face2,0.7
person1,body2,0.81
body1,voice2,0.9
face2,body1,0.6
voice3,body3,0.5
voice1,body2,0.5
```

## GRAPH

``` mermaid
graph LR

body3 ---|0.7| person2
person2 ---|0.9| face1
face1 ---|0.8| body2
face1 ---|0.2| person1
face1 ---|0.1| body1
person1 ---|0.7| face2
person1 ---|0.81| body2
body1 ---|0.9| voice2
face2 ---|0.6| body1
voice3 ---|0.5| body3
voice1 ---|0.5| body2
```

## OUTPUT

``` mermaid
graph LR

body3 ---|0.7| person2
person2 ---|0.9| face1
voice3 ---|0.5| body3


anon1 ---|0.4| body1
body1 ---|0.9| voice2
voice2 ---|0.4| anon1

body2 ---|0.81| person1
voice1 ---|0.5| body2
person1 ---|0.7| face2
person1 ---|0.405| voice1
```

# Step 2: body2 disappears

## INPUT

```
-body2
```

## GRAPH
``` mermaid
graph LR

body3 ---|0.7| person2
person2 ---|0.9| face1
person1 ---|0.7| face2
person1 ---|0.405| voice1
body1 ---|0.9| voice2
face2 ---|0.6| body1
voice3 ---|0.5| body3
```

## OUTPUT

``` mermaid
graph LR

voice1 ---|0.4| anon2

body3 ---|0.7| person2
person2 ---|0.9| face1
voice3 ---|0.5| body3
%% note: no direct connection between person2 and voice3 as likelihood below
%% threshold


face2 ---|0.7| person1
voice2 ---|0.9| body1
body1 ---|0.6| face2
body1 ---|0.42| person1
%% note: no direct connection between person1 and voice2 as likelihood below
%% threshold

```

# Step 3: body4 appears, associated to voice1

## INPUT

```
body4,voice1,0.6
```

## GRAPH
``` mermaid
graph LR

body3 ---|0.7| person2
person2 ---|0.9| face1
person1 ---|0.7| face2
person1 ---|0.405| voice1
body1 ---|0.9| voice2
body1 ---|0.42| person1
face2 ---|0.6| body1
voice3 ---|0.5| body3
voice1 ---|0.6| body4
```

## OUTPUT

``` mermaid
graph LR

%% body4 should be connected to existing anon2
voice1 ---|0.4| anon2
body4 ---|0.4| anon2
voice1 ---|0.6| body4

body3 ---|0.7| person2
person2 ---|0.9| face1
voice3 ---|0.5| body3


face2 ---|0.7| person1
voice2 ---|0.9| body1
body1 ---|0.6| face2
body1 ---|0.42| person1
```
