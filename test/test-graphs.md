# BASE

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
```

## Results:

``` mermaid
graph LR

body3 ---|0.7| person2
person2 ---|0.9| face1


person1 ---|0.7| face2
body1 ---|0.9| voice2
face2 ---|0.6| body1

body2 ---|0.4| anon_
```


# PERSON2 disappears

``` mermaid
graph LR

body3
face1 ---|0.8| body2
face1 ---|0.2| person1
face1 ---|0.1| body1
person1 ---|0.7| face2
person1 ---|0.81| body2
body1 ---|0.9| voice2
face2 ---|0.6| body1
```

## Results:

``` mermaid
graph LR

body3 ---|0.4| anon_

face1 ---|0.8| body2
body2 ---|0.4| anon_

person1 ---|0.7| face2
body1 ---|0.9| voice2
face2 ---|0.6| body1
```


