	?n??J4@?n??J4@!?n??J4@	=V㷄???=V㷄???!=V㷄???"e
=type.googleapis.com/tensorflow.profiler.PerGenericStepDetails$?n??J4@g??j+???A??Pk?4@Y??j+????*	gffffФ@2g
0Iterator::Model::Prefetch::FlatMap[0]::Generator??Q?@!?C??֧X@)??Q?@1?C??֧X@:Preprocessing2F
Iterator::Model???H??!a$?? ??)$????ۗ?1?&<c???:Preprocessing2P
Iterator::Model::Prefetch?? ?rh??!?C?5<k??)?? ?rh??1?C?5<k??:Preprocessing2Y
"Iterator::Model::Prefetch::FlatMapW?/?'@!n?????X@)n??t?1??_ױ???:Preprocessing:?
]Enqueuing data: you may want to combine small input data chunks into fewer but larger chunks.
?Data preprocessing: you may increase num_parallel_calls in <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#map" target="_blank">Dataset map()</a> or preprocess the data OFFLINE.
?Reading data from files in advance: you may tune parameters in the following tf.data API (<a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#prefetch" target="_blank">prefetch size</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#interleave" target="_blank">interleave cycle_length</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/TFRecordDataset#class_tfrecorddataset" target="_blank">reader buffer_size</a>)
?Reading data from files on demand: you should read data IN ADVANCE using the following tf.data API (<a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#prefetch" target="_blank">prefetch</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#interleave" target="_blank">interleave</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/TFRecordDataset#class_tfrecorddataset" target="_blank">reader buffer</a>)
?Other data reading or processing: you may consider using the <a href="https://www.tensorflow.org/programmers_guide/datasets" target="_blank">tf.data API</a> (if you are not using it now)?
:type.googleapis.com/tensorflow.profiler.BottleneckAnalysis?
device?Your program is NOT input-bound because only 0.8% of the total step time sampled is waiting for input. Therefore, you should focus on reducing other time.no*no9<V㷄???#You may skip the rest of this page.B?
@type.googleapis.com/tensorflow.profiler.GenericStepTimeBreakdown?
	g??j+???g??j+???!g??j+???      ??!       "      ??!       *      ??!       2	??Pk?4@??Pk?4@!??Pk?4@:      ??!       B      ??!       J	??j+??????j+????!??j+????R      ??!       Z	??j+??????j+????!??j+????JCPU_ONLYY<V㷄???b 