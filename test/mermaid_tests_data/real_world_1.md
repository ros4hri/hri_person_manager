
Notes:

- anonymous nodes must start with `anon`
- the main graph is kept and updated across tests steps

THRESHOLD: 0.4

# Step 1

## INPUT

```
f1
f2
b1
b2
v1
v3
f1,p1,0.8
f1,p2,0.7
f1,p3,0.5
f2,p4,0.8
f2,p2,0.65
b1,f1,0.8
b1,f2,0.75
b2,f2,0.7
b2,f1,0.65
v1,f1,0.6
v1,f2,0.58
v3,f1,0.2
v3,f2,0.21
```

## GRAPH

``` mermaid
graph LR

f1 ---|0.8| p1
f1 ---|0.7| p2
f1 ---|0.5| p3
f2 ---|0.8| p4
f2 ---|0.65| p2
b1 ---|0.8| f1
b1 ---|0.75| f2
b2 ---|0.7| f2
b2 ---|0.65| f1
v1 ---|0.6| f1
v1 ---|0.58| f2
v3 ---|0.2| f1
v3 ---|0.21| f2
```

## OUTPUT

``` mermaid
graph LR

f1 ---|0.8| p1
b1 ---|0.8| f1
v1 ---|0.6| f1
%% automatically computed
v1 ---|0.48| p1
b1 ---|0.64| p1

f2 ---|0.8| p4
b2 ---|0.7| f2
%% automatically computed
b2 ---|0.56| p4

p2
p3
v3 ---|0.4| anon1
```

# Step 2: face disappears

## INPUT

```
-f1
```

## GRAPH

``` mermaid
graph LR

f2 ---|0.8| p4
f2 ---|0.65| p2
b1 ---|0.75| f2
b2 ---|0.7| f2
v1 ---|0.58| f2

%% automatically computed
v1 ---|0.48| p1
b1 ---|0.64| p1
b2 ---|0.56| p4

p3
v3
```

## OUTPUT

``` mermaid
graph LR

b1 ---|0.64| p1

v1 ---|0.58| f2
v1 ---|0.464| p4
f2 ---|0.8| p4
b2 ---|0.7| f2
b2 ---|0.56| p4

p2
p3
v3 ---|0.4| anon1
```
