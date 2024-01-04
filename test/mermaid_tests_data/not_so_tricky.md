
Notes:

- anonymous nodes must start with `anon`
- the main graph is kept and updated across tests steps

THRESHOLD: 0.4

# Step 1

## INPUT

```
v1,f1,0.99
f1,p1,0.6
f2,p1,0.99
```

## GRAPH

```mermaid
v1 ---|0.99| f1
f1 ---|0.6| p1
f2 ---|0.99| p1
```

## OUTPUT

```mermaid
v1 ---|0.99| f1
f1 ---|0.4| anon1
v1 ---|0.4| anon1

f2 ---|0.99| p1
```
