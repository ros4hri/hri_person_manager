
Notes:

- anonymous nodes must start with `anon`
- the main graph is kept and updated across tests steps

THRESHOLD: 0.4

# Step 1: one independent feature

## INPUT

```
face1
```
## GRAPH

``` mermaid
graph LR

face1
```

## OUTPUT

``` mermaid
graph LR

face1 ---|0.4| anon1
```

# Step 2: two independent features

## INPUT

```
body1
```

## GRAPH

``` mermaid
graph LR

body1
face1
```

## OUTPUT

``` mermaid
graph LR

face1 ---|0.4| anon1
body1 ---|0.4| anon2
```

# Step 3: association between these features -- one anonymous person should disappear

## INPUT

```
face1,body1,0.6
```

## GRAPH

``` mermaid
graph LR

body1 ---|0.6| face1
```

## OUTPUT

``` mermaid
graph LR

%% anon2 is prefered to anon1 because body1 is alphabetically before face1
face1 ---|0.4| anon2
body1 ---|0.4| anon2
body1 ---|0.6| face1
```

# Step 4: feature removed

## INPUT

```
-face1
```

## GRAPH

``` mermaid
graph LR

body1
```

## OUTPUT

``` mermaid
graph LR

body1 ---|0.4| anon2
```

# Step 5: new face associated to person

## INPUT

```
face2,person1,0.7
```

## GRAPH

``` mermaid
graph LR

body1
face2 ---|0.7| person1
```

## OUTPUT

``` mermaid
graph LR

body1 ---|0.4| anon2
face2 ---|0.7| person1
```

# Step 6: adding feature to person via link

## INPUT

```
face2,body1,0.8
```

## GRAPH

``` mermaid
graph LR

face2 ---|0.8| body1
face2 ---|0.7| person1
```

## OUTPUT

``` mermaid
graph LR

face2 ---|0.8| body1
face2 ---|0.7| person1
%% here face2 -> person1 above 0.4 -> create the explicit link
body1 ---|0.56| person1
```

# Step 7: ensure stable association

## INPUT

```
-face2
```

## GRAPH

``` mermaid
graph LR

body1 ---|0.56| person1
```

## OUTPUT

``` mermaid
graph LR

body1 ---|0.56| person1
```

# Step 8: update computed association

## INPUT

```
face3,body1,0.8
face3,person1,0.6
```

## GRAPH

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
body1 ---|0.56| person1
```

## OUTPUT

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
%% as this relation is a 'computed' one, we update it
body1 ---|0.48| person1
```

# Step 9: new face

## INPUT

```
face4
```

## GRAPH

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
body1 ---|0.48| person1
face4
```

## OUTPUT

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
body1 ---|0.48| person1
face4 ---|0.4| anon3
```

# Step 10: new face being recognised

## INPUT

```
face4,person1,0.5
face4,person2,0.56
face4,person3,0.61
```

## GRAPH

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
body1 ---|0.48| person1
face4 ---|0.5| person1
face4 ---|0.56| person2
face4 ---|0.61| person3
```

## OUTPUT

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.6| person1
body1 ---|0.48| person1

face4 ---|0.61| person3

person2
```

# Step 11: first face recognition results updated

## INPUT

```
face3,person1,0.53
face3,person2,0.56
face3,person3,0.2
```

## GRAPH

``` mermaid
graph LR

face3 ---|0.8| body1
body1 ---|0.48| person1
face4 ---|0.5| person1

face4 ---|0.56| person2
face4 ---|0.61| person3

face3 ---|0.53| person1
face3 ---|0.56| person2
face3 ---|0.2| person3
```

## OUTPUT

``` mermaid
graph LR

face3 ---|0.8| body1
face3 ---|0.56| person2
person2 ---|0.448| body1

face4 ---|0.61| person3

person1
```
