/*  
 This script inserts reference and setup data into the world model
*/

-- Version of DB Schema
insert into app_info (db_version)
values ('2.1.17');

-- OBJECT Related Reference Data

insert into object_group (object_group_id, object_group)
values (1,'unknown'),
(2,'food'),
(3,'drinks'),
(4,'fruit'),
(5,'snacks'),
(6,'dishes'),
(7,'electronics'),
(8,'clothing'),
(9,'sports'),
(10,'transport'),
(11,'animal'),
(12,'bag'),
(13,'furniture'),
(99,'other');

insert into object_label (object_label_id, object_label) 
values (-1, 'unknown')
,(1,'person')
,(2,'bicycle')
,(3,'car')
,(4,'motorcycle')
,(5,'airplane')
,(6,'bus')
,(7,'train')
,(8,'truck')
,(9,'boat')
,(10,'traffic light')
,(11,'fire hydrant')
,(12,'stop sign')
,(13,'parking meter')
,(14,'bench')
,(15,'bird')
,(16,'cat')
,(17,'dog')
,(18,'horse')
,(19,'sheep')
,(20,'cow')
,(21,'elephant')
,(22,'bear')
,(23,'zebra')
,(24,'giraffe')
,(25,'backpack')
,(26,'umbrella')
,(27,'handbag')
,(28,'tie')
,(29,'suitcase')
,(30,'frisbee')
,(31,'skis')
,(32,'snowboard')
,(33,'sports ball')
,(34,'kite')
,(35,'baseball bat')
,(36,'baseball glove')
,(37,'skateboard')
,(38,'surfboard')
,(39,'tennis racket')
,(40,'bottle')
,(41,'wine glass')
,(42,'cup')
,(43,'fork')
,(44,'knife')
,(45,'spoon')
,(46,'bowl')
,(47,'banana')
,(48,'apple')
,(49,'sandwich')
,(50,'orange')
,(51,'broccoli')
,(52,'carrot')
,(53,'hot dog')
,(54,'pizza')
,(55,'donut')
,(56,'cake')
,(57,'chair')
,(58,'couch')
,(59,'potted plant')
,(60,'bed')
,(61,'dining table')
,(62,'toilet')
,(63,'tv')
,(64,'laptop')
,(65,'mouse')
,(66,'remote')
,(67,'keyboard')
,(68,'cell phone')
,(69,'microwave')
,(70,'oven')
,(71,'toaster')
,(72,'sink')
,(73,'refrigerator')
,(74,'book')
,(75,'clock')
,(76,'vase')
,(77,'scissors')
,(78,'teddy bear')
,(79,'hair drier')
,(80,'toothbrush');
	

-- generic object classes 

insert into object_class (object_class_id, name, decay_weight, object_group_id) 
values (-1,'not defefined', 0.5, 1),
(1,'person', 0.8, 11),
(2,'bicycle', 0.5, 10),
(3,'car', 0.8, 10),
(4,'motorcycle', 0.8, 10),
(5,'airplane', 0.1, 10),
(6,'bus', 0.8, 10),
(7,'train', 0.8, 10),
(8,'truck', 0.8, 10),
(9,'boat', 0.8, 10),
(10,'traffic light', 0.0, 99),
(11,'fire hydrant', 0.0, 99),
(12,'stop sign', 0.0, 99),
(13,'parking meter', 0.0, 99),
(14,'bench', 0.1, 13),
(15,'bird', 0.9, 11),
(16,'cat', 0.9, 11),
(17,'dog', 0.9, 11),
(18,'horse', 0.9, 11),
(19,'sheep', 0.9, 11),
(20,'cow', 0.9, 11),
(21,'elephant', 0.9, 11),
(22,'bear', 0.9, 11),
(23,'zebra', 0.9, 11),
(24,'giraffe', 0.9, 11),
(25,'backpack', 0.5, 12),
(26,'umbrella', 0.5, 99),
(27,'handbag', 0.5, 12),
(28,'tie', 0.5, 8),
(29,'suitcase', 0.5, 12),
(30,'frisbee', 0.5, 9),
(31,'skis', 0.5, 9),
(32,'snowboard', 0.5, 9),
(33,'sports ball', 0.5, 9),
(34,'kite', 0.5, 9),
(35,'baseball bat', 0.5, 9),
(36,'baseball glove', 0.5, 9),
(37,'skateboard', 0.5, 9),
(38,'surfboard', 0.5, 9),
(39,'tennis racket', 0.5, 9),
(40,'bottle', 0.7, 3),
(41,'wine glass', 0.7, 6),
(42,'cup', 0.7, 6),
(43,'fork', 0.7, 6),
(44,'knife', 0.7, 6),
(45,'spoon', 0.7, 6),
(46,'bowl', 0.7, 6),
(47,'banana', 0.7, 4),
(48,'apple', 0.7, 4),
(49,'sandwich', 0.7, 2),
(50,'orange', 0.7, 4),
(51,'broccoli', 0.7, 2),
(52,'carrot', 0.7, 2),
(53,'hot dog', 0.7, 2),
(54,'pizza', 0.7, 2),
(55,'donut', 0.7, 2),
(56,'cake', 0.7, 2),
(57,'chair', 0.8, 13),
(58,'couch', 0.2, 13),
(59,'potted plant', 0.2, 99),
(60,'bed', 0.05, 13),
(61,'dining table', 0.05, 4),
(62,'toilet', 0.0, 99),
(63,'tv', 0.1, 7),
(64,'laptop', 0.7, 7),
(65,'mouse', 0.9, 7),
(66,'remote', 0.9, 7),
(67,'keyboard', 0.9, 7),
(68,'cell phone', 0.9, 7),
(69,'microwave', 0.05, 7),
(70,'oven', 0.0, 7),
(71,'toaster', 0.3, 7),
(72,'sink', 0.0, 99),
(73,'refrigerator', 0.0, 7),
(74,'book', 0.7, 99),
(75,'clock', 0.5, 99),
(76,'vase', 0.5, 99),
(77,'scissors', 0.5, 99),
(78,'teddy bear', 0.5, 99),
(79,'hair drier', 0.8, 99),
(80,'toothbrush', 0.8, 99);

insert into object_class_label (object_class_id, object_label_id) 
values (-1,-1),
(1,1),
(2,2),
(3,3),
(4,4),
(5,5),
(6,6),
(7,7),
(8,8),
(9,9),
(10,10),
(11,11),
(12,12),
(13,13),
(14,14),
(15,15),
(16,16),
(17,17),
(18,18),
(19,19),
(20,20),
(21,21),
(22,22),
(23,23),
(24,24),
(25,25),
(26,26),
(27,27),
(28,28),
(29,29),
(30,30),
(31,31),
(32,32),
(33,33),
(34,34),
(35,35),
(36,36),
(37,37),
(38,38),
(39,39),
(40,40),
(41,41),
(42,42),
(43,43),
(44,44),
(45,45),
(46,46),
(47,47),
(48,48),
(49,49),
(50,50),
(51,51),
(52,52),
(53,53),
(54,54),
(55,55),
(56,56),
(57,57),
(58,58),
(59,59),
(60,60),
(61,61),
(62,62),
(63,63),
(64,64),
(65,65),
(66,66),
(67,67),
(68,68),
(69,69),
(70,70),
(71,71),
(72,72),
(73,73),
(74,74),
(75,75),
(76,76),
(77,77),
(78,78),
(79,79),
(80,80);

insert into object_size (size_id, size_desc) 
values (1,'tiny'),
 (2,'small'),
 (3,'medium'),
 (4,'large'),
 (5,'extra large');

insert into object_shape (shape_id, shape_desc) 
values (1,'box'),
 (2,'cylinder'),
 (3,'rectangular'),
 (4,'square'),
 (5,'circular'),
 (6,'trapezoid'),
 (7,'spere'),
 (8,'irregular'),
 (9,'other');
 
insert into object_relationship_type (object_relationship_type_id, object_relationship_type)
values (1,'supported by surface'),
(2,'next to'),
(3,'on top of'),
(4,'below'),
(5,'inside of');


insert into object_class_attribute_type (object_class_attribute_type_id, object_class_attribute_type)
values (1,'moveable'),
(2,'consumable');


-- AGENT Related Reference Data
insert into agent_attribute_type (agent_attribute_type_id, agent_attribute_type)
values (1, 'favourite drink');

insert into agent_feature_type (agent_feature_type_id, agent_feature_type)
values (1, 'face'),
(2, 'shirt'),
(3, 'pants'),
(4, 'head');


insert into object_sub_class (object_sub_class_id, object_class_id, object_group_id, name)
values (1, 40, 3, 'water'),
(2, 40, 3, 'big_coke'),
(3, 40, 3, 'ice_tea'),
(4, 40, 3, 'cola'),
(5, 40, 3, 'fanta'),
(6, 40, 2, 'curry'),
(7, 40, 2, 'mayonaise'),
(8, 40, 2, 'suasages'),
(9, 42, 2, 'pea_soup'),
(10, 40, 5, 'pringles');


